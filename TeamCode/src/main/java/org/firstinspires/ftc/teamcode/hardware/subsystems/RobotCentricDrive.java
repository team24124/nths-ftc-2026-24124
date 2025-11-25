package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.util.ArraySelect;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;

public class RobotCentricDrive extends Drivetrain implements TelemetryObservable {
    public RobotCentricDrive(HardwareMap hw, Pose2d start) {
        super(hw, start);
    }

    /**
     * Drive Train implementation from this <a href="https://www.youtube.com/watch?v=gnSW2QpkGXQ">video</a>
     * @param x Amount of x (Ex. left/right on the left joystick)
     * @param y Amount of y (Ex. up/down on the left joystick)
     * @param rx Amount of turn (Ex. left/right on the right joystick)
     */
    @Override
    public void drive(double x, double y, double rx, boolean align) {
        double voltage = voltageSensor.getVoltage();
        if (align) {
            rx = -thetaPD.calculate(rx, 0, voltage);
        }

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftPower = power * cos / max + rx;
        double leftBackPower = power * sin / max + rx;
        double rightPower = power * sin / max - rx;
        double rightBackPower = power * cos / max - rx;

        if ((power + Math.abs(rx)) > 1) {
            double denominator = power + Math.abs(rx);
            if (align) denominator += 0.2;
            leftPower /= denominator;
            leftBackPower /= denominator;
            rightPower /= denominator;
            rightBackPower /= denominator;
        }

        ArraySelect<Double> speeds = getSpeeds();
        if (align) {
            super.setDrivePowers(
                    leftPower * 0.5,
                    rightPower * 0.5,
                    leftBackPower * 0.5,
                    rightBackPower * 0.5
            );
        } else {
            super.setDrivePowers(
                    leftPower * speeds.getSelected(),
                    rightPower * speeds.getSelected(),
                    leftBackPower * speeds.getSelected(),
                    rightBackPower * speeds.getSelected()
            );
        }
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
        telemetry.addData("Speed", getSpeeds().getSelected());
        telemetry.addData("Heading (rad)", current.heading.toDouble());
        telemetry.addData("Heading (°)", Math.toDegrees(current.heading.toDouble()));
    }
}
