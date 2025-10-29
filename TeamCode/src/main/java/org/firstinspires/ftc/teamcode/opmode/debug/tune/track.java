package org.firstinspires.ftc.teamcode.opmode.debug.tune;

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
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;

import java.util.List;

@TeleOp(name = "track", group = "MI BOMBO")
public class track extends OpMode {
    private List<LynxModule> hubs;
    private VoltageSensor voltageSensor;
    private Drivetrain drivetrain;
    private Limelight limelight;
    private GamepadEx driver;

    private boolean alignToAT = false;
    private PIDF thetaPD = new PIDF();
    private PIDF distPD = new PIDF();

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        drivetrain = new RobotCentricDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));
        driver = new GamepadEx(gamepad1);
        limelight = new Limelight(hardwareMap);

        thetaPD.setPD(3, 0.1, 0.7);
        distPD.setPD(0.08, 0.00009, 0.7);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

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
                drivetrain.drive(0, -distPD.calculate(limelight.distance() - 40, 0, voltageSensor.getVoltage()), -thetaPD.calculate(Math.toRadians(limelight.degreeOffset()), 0, voltageSensor.getVoltage()), false);
            } else {
                drivetrain.drive(x, y, rx, false);
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