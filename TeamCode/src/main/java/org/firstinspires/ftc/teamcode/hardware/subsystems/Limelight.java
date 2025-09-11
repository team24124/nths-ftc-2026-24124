package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardware.SubsystemBase;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryObservable;

import java.util.List;

public class Limelight implements SubsystemBase, TelemetryObservable {
    private final Limelight3A limelight;
    public enum Pipeline {
        PSS1(0), // First python snapscript
        AT1(1), // Obelisk detection
        AT2(2), // 2D goal detection
        AT3(3), // Full 3D goal detection
        AT4(4); // MT2 for pose reset

        public final int pipelineNum;

        Pipeline(int pipelineNum) {
            this.pipelineNum = pipelineNum;
        }
    }

    private Pipeline pipeline;

    private final double cameraAngle = 0; // Facing forward == 0 degrees

    public Limelight(HardwareMap hw){
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(Pipeline.AT1.pipelineNum);
        limelight.setPollRateHz(100); // Ask Limelight for data (100 times per second)
        limelight.start();
        pipeline = Pipeline.AT1;
    }

    // Switch pipeline if input is not the same as pipeline
    public void setPipeline(Pipeline pipeline) {
        if (pipeline != this.pipeline) {
            limelight.pipelineSwitch(pipeline.pipelineNum);
            this.pipeline = pipeline;
        }
    }

    // Checks python output & validity
    public boolean isDetected() {
        return getResult().isValid();
    }

    private LLResultTypes.FiducialResult getFiducial() {
        List<LLResultTypes.FiducialResult> fiducials = getResult().getFiducialResults();
        if (!fiducials.isEmpty()) {
            return fiducials.get(0); // Gets first detected tag in the list of tags of fiducials
        } else {
            return null;
        }
    }

    // AT type to determine Obelisk pattern (use for state machine within autonomous)
    public int ATType() {
        return getFiducial().getFiducialId();
    }

    private LLResult getResult() { return limelight.getLatestResult(); }

    //---------------------------------- values for PIDF ----------------------------------

    // Tx for rotation
    public double degreeOffset() {
        return getResult().getTx();
    }

    // Distance in inches
    public double distance() {
        return 8 + Math.tan(Math.toRadians(cameraAngle + getResult().getTyNC())) * 20; // +8 to center limelight to center of robot
    }

    //---------------------------------- target position returners ----------------------------------

    // Field space X and Y offset for detected
    public Pose2d targetVectorFieldSpace(Pose2d botPose) {
        // Target raw data
        double ty = Math.tan(Math.toRadians(cameraAngle + getResult().getTyNC())) * 20;
        double distanceFromCam = Math.sqrt(Math.pow(ty, 2) + Math.pow(20, 2));
        double tx = Math.tan(Math.toRadians(0 + getResult().getTxNC())) * distanceFromCam; // +0 for limelight yaw

        // Target offset
        double x = tx - 13;
        double y = ty - 19;

        double heading = botPose.heading.toDouble();

        // Target converted into field centric
        double x1 = botPose.position.x + (x * Math.cos(heading) - y * Math.sin(heading));
        double y1 = botPose.position.y + (x * Math.sin(heading) + y * Math.cos(heading));

        return new Pose2d(x1, y1, heading);
    }

    // Field space X and Y offset for apriltag
    public Pose2d ATTargetVectorFieldSpace(Pose2d botPose) {
        Pose3D targetPose = getFiducial().getTargetPoseRobotSpace();

        double y = targetPose.getPosition().x * 39.37 - 50; // Robot space is cartesian with X+ pointing forward and Y+ pointing right
        double x = targetPose.getPosition().y * 39.37 - 0;

        double heading = botPose.heading.toDouble();

        double x1 = botPose.position.x + (x * Math.cos(heading) - y * Math.sin(heading));
        double y1 = botPose.position.y + (x * Math.sin(heading) + y * Math.cos(heading));

        return new Pose2d(x1, y1, heading);
    }

    // Robot space X and Y offset and heading for apriltag
    public Pose2d ATTargetPoseFieldSpace(Pose2d botPose) {
        Pose3D pose = getFiducial().getTargetPoseRobotSpace();
        double theta = Math.toRadians(degreeOffset());

        double x = pose.getPosition().y * 39.37;
        double y = pose.getPosition().x * 39.37 - 50; // Offset target

        double heading = botPose.heading.toDouble();

        double x1 = botPose.position.x + (x * Math.cos(heading) - y * Math.sin(heading));
        double y1 = botPose.position.y + (x * Math.sin(heading) + y * Math.cos(heading));

        return new Pose2d(x1, y1, (heading + (2 * Math.PI - theta)) % (2 * Math.PI)); // Combine heading and rotation needed
    }

    // Field space bot pose (this is mapped, face desired AT ID 20 or 24)
    public Pose2d ATRobotPoseFieldSpace() {
        Pose3D pose3d = getResult().getBotpose_MT2();
        return new Pose2d(pose3d.getPosition().x * 39.37, pose3d.getPosition().y * 39.37, pose3d.getOrientation().getYaw(AngleUnit.RADIANS) + 180);
    }

    //---------------------------------- telemetry ----------------------------------

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        LLResult result = getResult();
        telemetry.addData("Status", limelight.getStatus().toString());
        telemetry.addData("pipeline", pipeline.name());
        if (getResult().isValid() && result.getPythonOutput().length > 2) {
            double[] pythonOutputs = result.getPythonOutput();
            telemetry.addData("validity", pythonOutputs[0]);
            telemetry.addData("targetX", Math.round(pythonOutputs[1]));
            telemetry.addData("targetY", Math.round(pythonOutputs[2]));
            telemetry.addData("angle", Math.round(pythonOutputs[3]));
            telemetry.addData("area %", Math.round(pythonOutputs[4]));
        } else if (getResult().isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getTargetPoseRobotSpace(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
        }
        telemetry.update();
    }

    @Override
    public String getName() {
        return "Limelight";
    }
}
