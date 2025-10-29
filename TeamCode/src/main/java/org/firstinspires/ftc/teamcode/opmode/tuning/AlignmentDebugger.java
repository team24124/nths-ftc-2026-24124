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
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;

import java.util.List;

@Config
@TeleOp(name = "Align 3A", group = "tuning")
public class AlignmentDebugger extends OpMode {
    private List<LynxModule> hubs;
    private VoltageSensor voltageSensor;
    private Drivetrain drivetrain;
    private TeleOpTrajectories trajectories;
    private Limelight limelight;
    private GamepadEx driver;

    // --- Tune theta PD ---
    public static double Kp = 3;
    public static double Kd = 0.1;
    public static double sf = 0.7;

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
        drivetrain = new FieldCentricDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));
        trajectories = TeleOpTrajectories.INSTANCE;
        driver = new GamepadEx(gamepad1);
        limelight = new Limelight(hardwareMap);

        thetaPD.setPD(Kp, Kd, sf);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        thetaPD.setPD(Kp, Kd, sf);

        double y = driver.getLeftY();
        double x = driver.getLeftX();
        double rx = driver.getRightX();

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            limelight.setPipeline(Limelight.Pipeline.AT1);
            alignToAT = true;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            alignToAT = false;
        }

        if (alignToAT) {
            if (limelight.isDetected()) {
                drivetrain.drive(x, y, -thetaPD.calculate(Math.toRadians(limelight.degreeOffset()), 0, voltageSensor.getVoltage()), false);
            } else {
                drivetrain.drive(x, y, -thetaPD.calculate(trajectories.theta(drivetrain, 72, 72), 0, voltageSensor.getVoltage()), false);
            }
        } else {
            drivetrain.drive(x, y, rx, false);
        }
        drivetrain.periodic();

        driver.readButtons();

        telemetry.addData("\nAlign", alignToAT);
        telemetry.addData("\nIs Detected", limelight.isDetected());
        telemetry.addData("\nX", "%.1f", drivetrain.getPosition().component1().x);
        telemetry.addData("Y", "%.1f", drivetrain.getPosition().component1().y);
        telemetry.addData("Heading", "%.1f", drivetrain.getPosition().heading.toDouble());
        telemetry.update();
    }
}