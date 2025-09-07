package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.util.ArraySelect;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryObservable;

public class RobotCentricDrive extends Drivetrain implements TelemetryObservable {
    public RobotCentricDrive(HardwareMap hw, Pose2d start) {
        super(hw, start);
    }

    /**
     * Drive Train implementation from this <a href="https://www.youtube.com/watch?v=gnSW2QpkGXQ">video</a>
     * @param x Amount of x (Ex. left/right on the left joystick)
     * @param y Amount of y (Ex. up/down on the left joystick)
     * @param turn Amount of turn (Ex. left/right on the right joystick)
     */
    @Override
    public void drive(double x, double y, double turn) {
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftPower = power * cos / max + turn;
        double leftBackPower = power * sin / max + turn;
        double rightPower = power * sin / max - turn;
        double rightBackPower = power * cos / max - turn;

        if ((power + Math.abs(turn)) > 1) {
            leftPower /= power + Math.abs(turn);
            leftBackPower /= power + Math.abs(turn);
            rightPower /= power + Math.abs(turn);
            rightBackPower /= power + Math.abs(turn);
        }

        ArraySelect<Double> speeds = getSpeeds();
        super.setDrivePowers(
                leftPower * speeds.getSelected(),
                rightPower * speeds.getSelected(),
                leftBackPower * speeds.getSelected(),
                rightBackPower * speeds.getSelected()
        );
    }

    @Override
    public void align(double x, double y, double dist, double theta) { // Method called during alignment mode
        double voltage = voltageSensor.getVoltage();
        double rx = thetaPID.calculate(theta, 0, voltage);
        if (!Utilities.isBetween(dist, 60, 108) && dist <= 60) { // Utilize gamepad1 y if inside threshold
            y = distancePID.calculate(dist, 61, voltage);
        } else if (!Utilities.isBetween(dist, 60, 108) && dist >= 108) {
            y = distancePID.calculate(dist, 107, voltage);
        }
        double botHeading = getDrive().localizer.getPose().heading.toDouble();

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        ArraySelect<Double> speeds = getSpeeds();
        super.setDrivePowers(
                frontLeftPower * speeds.getSelected(),
                frontRightPower * speeds.getSelected(),
                backLeftPower * speeds.getSelected(),
                backRightPower * speeds.getSelected()
        );
    }

    @Override
    public void periodic(){
        getDrive().updatePoseEstimate();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        Pose2d current = getDrive().localizer.getPose();

        telemetry.addData("X", current.position.x);
        telemetry.addData("Y", current.position.y);
        telemetry.addData("Heading (°)", current.heading.toDouble());
        telemetry.addData("Heading (°)", Math.toRadians(current.heading.toDouble()));
    }
}
