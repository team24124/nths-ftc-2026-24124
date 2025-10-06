package org.firstinspires.ftc.teamcode.opmode.tuning;

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
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpTrajectories;
import org.firstinspires.ftc.teamcode.util.ArraySelect;
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;

import java.util.List;

@Config
@TeleOp(name = "AlignmentDebugger", group = "test")
public class AlignmentDebugger extends OpMode {
    private List<LynxModule> hubs;
    private VoltageSensor voltageSensor;
    private Drivetrain driveTrain;
    private TeleOpTrajectories trajectories;
    private Limelight limelight;
    private GamepadEx driver;

    // --- Tune theta PD ---
    public static double Kp = 0.0000001;
    public static double Kd = 0.0000001;
    public static double a = 0.0000001;

    // --- PD ---
    private boolean alignToAT = false;
    private PIDF thetaPD = new PIDF();

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        driveTrain = new FieldCentricDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));
        trajectories = TeleOpTrajectories.INSTANCE;
        driver = new GamepadEx(gamepad1);
        limelight = new Limelight(hardwareMap);

        thetaPD.setPD(Kp, Kd, a);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        thetaPD.setPD(Kp, Kd, a);

        double y = driver.getLeftY();
        double x = driver.getLeftX();
        double rx = driver.getRightX();

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            limelight.setPipeline(Limelight.Pipeline.AT2);
            alignToAT = true;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            alignToAT = false;
        }

        if (alignToAT) {
            if (limelight.isDetected()) {
                align(x, y, limelight.degreeOffset()); // Private align method to avoid using DriveTrain PD
            } else {
                driveTrain.drive(x, y, trajectories.rotation(driveTrain, -72), false);
            }
        } else {
            driveTrain.drive(x, y, rx, false);
        }
        driveTrain.periodic();

        driver.readButtons();

        telemetry.addData("\nAlign", alignToAT);
        telemetry.addData("\nDetected", limelight.isDetected());
        telemetry.addData("\nTurn", trajectories.rotation(driveTrain, -72));
        telemetry.addData("\nPose", driveTrain.getPosition());
        telemetry.update();
    }

    public void align(double x, double y, double theta) {
        double voltage = voltageSensor.getVoltage();
        double botHeading = (driveTrain.getHeading() + Math.PI/2) % (Math.PI*2);

        double rotX = x * Math.cos(botHeading) + y * Math.sin(botHeading);
        double rotY = -x * Math.sin(botHeading) + y * Math.cos(botHeading);
        rotX = rotX * 1.1;

        double rx = thetaPD.calculate(theta, 0, voltage);

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