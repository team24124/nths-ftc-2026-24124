package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.ArraySelect;
import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.util.Utilities;

import java.util.List;

@Config
@TeleOp(name = "AlignmentDebugger", group = "test")
public class AlignmentDebugger extends OpMode {
    private List<LynxModule> hubs;
    private VoltageSensor voltageSensor;
    private Drivetrain driveTrain;
    private Limelight limelight;
    private GamepadEx driver;

    // --- tune distance PID ---
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double a = 0;
    public static double integralSumLimit = 0;

    // --- tune theta PID
    public static double tKp = 0;
    public static double tKi = 0;
    public static double tKd = 0;
    public static double ta = 0;
    public static double tIntegralSumLimit = 0;

    // --- PIDs ---
    private boolean alignToAT = false;
    private PIDF thetaPID = new PIDF();
    private PIDF distancePID = new PIDF();

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        driveTrain = new FieldCentricDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));
        driver = new GamepadEx(gamepad1);
        limelight = new Limelight(hardwareMap);

        distancePID.setPID(Kp, Ki, Kd, a, integralSumLimit);
        thetaPID.setPID(tKp, tKi, tKd, ta, tIntegralSumLimit);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        distancePID.setPID(Kp, Ki, Kd, a, integralSumLimit);
        thetaPID.setPID(tKp, tKi, tKd, ta, tIntegralSumLimit);

        double y = Math.abs(-driver.getLeftY()) > 0.05 ? -driver.getLeftY() : 0;
        double x = Math.abs(driver.getLeftX()) > 0.05 ? driver.getLeftX() : 0;
        double rx = Math.abs(-driver.getRightX()) > 0.05 ? -driver.getRightX() : 0;

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            limelight.setPipeline(Limelight.Pipeline.AT1);
            alignToAT = true;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            alignToAT = false;
        }

        driver.readButtons();
        driveTrain.periodic();

        if (!driveTrain.getDrive().isBusy) {
            if (alignToAT) {
                if (limelight.isDetected()) {
                    align(x, y, limelight.distance(), limelight.degreeOffset()); // Private align method to avoid using DriveTrain PIDs
                } else {
                    driveTrain.drive(x, y, 0.75);
                }
            } else {
                driveTrain.drive(x, y, rx);
            }
        }
    }

    public void align(double x, double y, double dist, double theta) {
        double voltage = voltageSensor.getVoltage();
        double botHeading = driveTrain.getDrive().localizer.getPose().heading.toDouble();
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

        ArraySelect<Double> speeds = driveTrain.getSpeeds();
        driveTrain.setDrivePowers(
                frontLeftPower * speeds.getSelected(),
                frontRightPower * speeds.getSelected(),
                backLeftPower * speeds.getSelected(),
                backRightPower * speeds.getSelected()
        );
    }
}