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
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDrive;
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
        driveTrain = new RobotCentricDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));
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
        
        double y = -driver.getLeftY();
        double x = driver.getLeftX();
        double rx = -driver.getRightX();

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
            if (limelight.isDetected() && alignToAT) {
                align(x, y, limelight.distance(), limelight.degreeOffset()); // Private align method to avoid using DriveTrain PIDs
            } else {
                driveTrain.drive(x, y, rx);
            }
        }
    }

    private void align(double x, double y, double dist, double theta) {
        double voltage = voltageSensor.getVoltage();
        double rx = thetaPID.calculate(theta, 0, voltage);
        if (!Utilities.isBetween(dist, 60, 108) && dist <= 60) {
            y = distancePID.calculate(dist, 61, voltage);
        } else if (!Utilities.isBetween(dist, 60, 108) && dist >= 108) {
            y = distancePID.calculate(dist, 107, voltage);
        }
        double botHeading = driveTrain.getHeading();

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        ArraySelect<Double> speeds = driveTrain.getSpeeds();
        driveTrain.setDrivePowers(
                frontLeftPower * speeds.getSelected(),
                frontRightPower * speeds.getSelected(),
                backLeftPower * speeds.getSelected(),
                backRightPower * speeds.getSelected()
        );
    }
}