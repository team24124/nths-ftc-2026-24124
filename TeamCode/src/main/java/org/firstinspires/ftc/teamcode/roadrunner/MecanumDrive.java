package org.firstinspires.ftc.teamcode.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hidden.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.hidden.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.hidden.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.hidden.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.hidden.roadrunnerNoTouch.Drawing;
import org.firstinspires.ftc.teamcode.hidden.roadrunnerNoTouch.Localizer;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDrive {
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        // drive model parameters
        public double inPerTick = 0.0019599718228152;
        public double lateralInPerTick = 0.0016043990885991073;
        public double trackWidthTicks = 6256.400931030658;

        // feedforward parameters (in tick units)
        public double kS = 0.7724322803721293;
        public double kV = 0.0003870687078841431;
        public double kA = 0.00003;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 0.0;
        public double lateralGain = 0.0;
        public double headingGain = 0.0; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn
    }

    public static Params PARAMS = new Params();

    public boolean isBusy = false; // Is robot performing a movement

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

    public final LazyImu lazyImu;

    public final Localizer localizer;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftBack, rightBack, rightFront;
        public final IMU imu;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;
        private boolean initialized;
        private Pose2d pose;

        public DriveLocalizer(Pose2d pose) {
            leftFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftFront));
            leftBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftBack));
            rightBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightBack));
            rightFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightFront));

            imu = lazyImu.get();

            // TODO: reverse encoders if needed
            //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

            this.pose = pose;
        }

        @Override
        public void setPose(Pose2d pose) {
            this.pose = pose;
        }

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public PoseVelocity2d update() {
            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            if (!initialized) {
                initialized = true;

                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;

                lastHeading = heading;

                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            }

            double headingDelta = heading.minus(lastHeading);
            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            pose = pose.plus(new Twist2d(
                    twist.line.value(),
                    headingDelta
            ));

            return twist.velocity().value();
        }
    }

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new PinpointLocalizer(hardwareMap, PARAMS.inPerTick, pose);

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        private boolean ECT;

        public FollowTrajectoryAction(TimeTrajectory t, boolean ECT) {
            timeTrajectory = t;

            this.ECT = ECT;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }
            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());

            isBusy = true;

            if (ECT && t >= timeTrajectory.duration && error.position.norm() < 1.5 && robotVelRobot.linearVel.norm() < 0.4 || t >= timeTrajectory.duration + 2) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                isBusy = false;
                return false;
            } else if (!ECT && t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                isBusy = false;
                return false;
            }

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            isBusy = true;

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                isBusy = false;
                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());
        
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));
        
        
        return vel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose, boolean enableTrajectoryExtraCorrection) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                t -> new FollowTrajectoryAction(t, enableTrajectoryExtraCorrection), // Second parameter is used to enable/disable extra correction (somewhat disables the end of trajectory using time and instead uses distance)
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }


    // EXPERIMENTAL
    // Alternate trajectory follower for roadrunner using displacement trajectories (distance instead of time)
    // to use, put at the bottom of RR 1.0's MecanumDrive file, and change actionBuilder to use it instead of FollowTrajectoryAction
    // Created by j5155 from team 12087 based on https://rr.brott.dev/docs/v1-0/guides/path-following/
    // Licensed under the BSD 3-Clause Clear License
    // If you use this, I would love to know how it goes/what issues you encounter, I'm @j5155 on discord
//    public final class FollowTrajectoryAsPathAction implements Action {
//        public final DisplacementTrajectory dispTraj;
//        public final HolonomicController contr;
//
//        private final double[] xPoints, yPoints;
//        double disp; // displacement; target distance traveled in path
//
//        // only used for recording what end time should be
//        // to avoid the dreaded wiggle
//        public final ElapsedTime trajectoryRunningTime = new ElapsedTime();
//        public double targetTimeSeconds;
//        boolean initialized = false;
//
//        public FollowTrajectoryAsPathAction(TimeTrajectory t) {
//            dispTraj = new DisplacementTrajectory(t.path, t.profile.dispProfile);
//            contr = new HolonomicController( // PD to point/velocity controller
//                    PARAMS.axialGain,
//                    PARAMS.lateralGain,
//                    PARAMS.headingGain,
//                    PARAMS.axialVelGain,
//                    PARAMS.lateralVelGain,
//                    PARAMS.headingVelGain);
//            disp = 0;
//
//            targetTimeSeconds = t.duration;
//
//
//            // ONLY USED FOR PREVIEW
//            List<Double> disps = com.acmerobotics.roadrunner.Math.range( // returns evenly spaced values
//                    0,  // between 0 and the length of the path
//                    dispTraj.path.length(),
//                    Math.max(2, // minimum 2
//                            (int) Math.ceil(dispTraj.path.length() / 2) // max total of half the length of the path
//                    ));
//            // so really make 1 sample every 2 inches (I think)
//
//            // and then convert them into lists of doubles of x and y so they can be shown on dash
//            xPoints = new double[disps.size()];
//            yPoints = new double[disps.size()];
//            for (int i = 0; i < disps.size(); i++) {
//                Pose2d p = t.path.get(disps.get(i), 1).value();
//                xPoints[i] = p.position.x;
//                yPoints[i] = p.position.y;
//            }
//
//
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket p) {
//            // needs to only run once
//            // idk if this is the most elegant solution
//            if (!initialized) {
//                trajectoryRunningTime.reset();
//                initialized = true;
//            }
//
//
//            PoseVelocity2d robotVelRobot = updatePoseEstimate();
//
//            // find the closest position on the path to the robot's current position
//            // (using binary search? I think? project function is hard to understand)
//            // where "position on the path" is represent as disp or distance into the path
//            // so like for a 10 inch long path, if disp was 5 it would be halfway along the path
//            disp = dispTraj.project(localizer.getPose().position, disp);
//
//
//            // check if the trajectory should end
//            // this logic is pretty much made up and doesnt really make sense
//            // and it wiggles occasionally
//            // but it does usually work
//            p.addLine("dispTraj position minus current " + dispTraj.get(dispTraj.length()).position.value().minus(localizer.getPose().position).norm());
//            p.addLine("disp " + disp + " dispTraj length" + dispTraj.length());
//
//            // if robot within 2 in of end pose
//            if (dispTraj.get(dispTraj.length()).position.value().minus(localizer.getPose().position).norm() < 2
//                    // or the closest position on the path is less then 2 inches away from the end of the path
//                    || (disp + 2) >= dispTraj.length()
//                    // or the trajectory has been running for 1 second more then it's suppposed to (this 1 second is weird)
//                    || (trajectoryRunningTime.seconds() >= targetTimeSeconds + 1)) {
//
//                // stop all the motors
//                leftFront.setPower(0);
//                leftBack.setPower(0);
//                rightBack.setPower(0);
//                rightFront.setPower(0);
//
//                // end the action
//                return false;
//            }
//
//            // ok so the trajectory shouldn't end yet
//
//            // at the start of the trajectory the mped velocity is probably 0
//            // so...never target the start of the path
//            // this should also hopefully boost accel a little bit
//            if (disp < 2) {
//                disp = 2;
//            }
//
//            // find the target pose and vel of the closest point on the path
//            Pose2dDual<Time> poseTarget = dispTraj.get(disp);
//
//            // calculate the command based on PD on the target pose and vel
//            PoseVelocity2dDual<Time> cmd = contr.compute(poseTarget, localizer.getPose(), robotVelRobot);
//
//            // convert it into wheel velocities with inverse kinematics
//            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(cmd);
//            // find voltage for voltage compensation
//            double voltage = voltageSensor.getVoltage();
//
//            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, 0); // kA 0; ignore acceleration
//
//            // calculate the volts to send to each wheel based on the target velocity for the wheel
//            // divide it by the current voltage to get the power from 0-1
//            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
//            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
//            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
//            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);
//
//            // log target to rr logs
//            FlightRecorder.write("TARGET_POSE", new PoseMessage(poseTarget.value()));
//
//            // show dash data
//            p.put("x", localizer.getPose().position.x);
//            p.put("y", localizer.getPose().position.y);
//            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.log()));
//
//            Pose2d error = poseTarget.value().minusExp(localizer.getPose());
//            p.put("xError", error.position.x);
//            p.put("yError", error.position.y);
//            p.put("headingError (deg)", Math.toDegrees(error.heading.log()));
//
//            // only draw when active; only one drive action should be active at a time
//            Canvas c = p.fieldOverlay();
//            drawPoseHistory(c);
//
//            c.setStroke("#4CAF50");
//            Drawing.drawRobot(c, poseTarget.value());
//
//            c.setStroke("#3F51B5");
//            Drawing.drawRobot(c, localizer.getPose());
//
//            c.setStroke("#4CAF50FF");
//            c.setStrokeWidth(1);
//            c.strokePolyline(xPoints, yPoints);
//
//            // continue running the action
//            return true;
//
//        }
//
//        @Override
//        public void preview(Canvas c) {
//            c.setStroke("#4CAF507A");
//            c.setStrokeWidth(1);
//            c.strokePolyline(xPoints, yPoints);
//        }
//    }
}
