package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.util.ArraySelect;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;

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
    public void drive(double x, double y, double rx, boolean align) {
        double voltage = voltageSensor.getVoltage();
        double botHeading = (getHeading() + Math.PI/2) % (Math.PI*2);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(botHeading) + y * Math.sin(botHeading);
        double rotY = -x * Math.sin(botHeading) + y * Math.cos(botHeading);
        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator;

        if (!align) {
            /*
             * Denominator is the largest motor power (absolute value) or 1
             * This ensures all the powers maintain the same ratio,
             * but only if at least one is out of the range [-1, 1]
             */
            denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        } else {
            rx = thetaPD.calculate(rx, 0, voltage); // rx is limelight input
            denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.2);
        }

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
        telemetry.addData("Heading (rad)", current.heading.toDouble());
        telemetry.addData("Heading (°)", Math.toDegrees(current.heading.toDouble()));
    }
}
