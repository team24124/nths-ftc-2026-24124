package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.util.ArraySelect;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryObservable;

public class FieldCentricDrive extends Drivetrain implements TelemetryObservable {
    public FieldCentricDrive(HardwareMap hw, Pose2d start) {
        super(hw, start);
    }

    /**
     * Drive Train implementation from this <a href="https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric-final-sample-code">guide</a>
     * @param x Amount of x (Ex. left/right on the left joystick)
     * @param y Amount of y (Ex. up/down on the left joystick)
     * @param rx Amount of turn (Ex. left/right on the right joystick)
     */
    @Override
    public void drive(double x, double y, double rx) {
        double botHeading = getDrive().localizer.getPose().heading.toDouble();

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        /*
         * Denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
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
    public void align(double x, double y, double dist, double theta) {
        double voltage = voltageSensor.getVoltage();
        double botHeading = getDrive().localizer.getPose().heading.toDouble();
        double forwardPower = 0;
        double rx = thetaPID.calculate(theta, 0, voltage);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        rotX = rotX * 1.1;

        if (!Utilities.isBetween(dist, 60, 108)) {
            if (dist < 60) {
                forwardPower = distancePID.calculate(dist, 62, voltage);
            } else if (dist > 108) {
                forwardPower = distancePID.calculate(dist, 107, voltage);
            }

            double denominator = Math.max(Math.abs(forwardPower) + Math.abs(rx), 1);
            frontLeftPower = (forwardPower + rx) / denominator;
            backLeftPower = (forwardPower + rx) / denominator;
            frontRightPower = (forwardPower - rx) / denominator;
            backRightPower = (forwardPower - rx) / denominator;
        } else {
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 2);
            frontLeftPower = (rotY + rotX + rx) / denominator;
            backLeftPower = (rotY - rotX + rx) / denominator;
            frontRightPower = (rotY - rotX - rx) / denominator;
            backRightPower = (rotY + rotX - rx) / denominator;
        }

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
        telemetry.addData("Heading (rad)", current.heading.toDouble());
        telemetry.addData("Heading (°)", Math.toDegrees(current.heading.toDouble()));
    }
}
