package org.firstinspires.ftc.teamcode.opmode.debug.test;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.REVColourV3;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.TelemetryControl;

import java.util.List;
import java.util.Objects;

@TeleOp(name = "Components", group = "test")
public class ComponentDebugger extends OpMode {
    private List<LynxModule> hubs;
    private GamepadEx driver;
    private ActionScheduler actions;
    private Drivetrain drivetrain;
    public Flywheel flywheel;
    public Intake intake;
    public REVColourV3 colorSensor;
    public Spindexer spindexer;
    private TelemetryControl telemetryControl;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        driver = new GamepadEx(gamepad1);
        actions = ActionScheduler.INSTANCE;
        actions.init();
        drivetrain = new RobotCentricDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));
        spindexer = new Spindexer(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);
        colorSensor = new REVColourV3(hardwareMap);
        telemetryControl = new TelemetryControl(telemetry);
        telemetryControl
                .subscribe(drivetrain)
                .subscribe(spindexer)
                .subscribe(flywheel)
                .subscribe(intake)
                .subscribe(colorSensor)
                .subscribe(actions);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        double y = driver.getLeftY();
        double x = driver.getLeftX();
        double rx = driver.getRightX();

        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            drivetrain.toggleSpeeds();
        }

        if (driver.isDown(GamepadKeys.Button.A)) {
            actions.schedule(flywheel.runFlywheel());
        } else {
            actions.schedule(flywheel.stopFlywheel());
        }

        if (driver.isDown(GamepadKeys.Button.X)) {
            actions.schedule(intakePeriodic());
        } else {
            actions.schedule(intake.stopIntake());
        }

        if (driver.wasJustPressed(GamepadKeys.Button.Y) && flywheel.primed) {
            actions.schedule(spindexer.shootOne());
        }
        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) && flywheel.primed && spindexer.slots.contains("purple")) {
            actions.schedule(new SequentialAction(
                    spindexer.sortTo(spindexer.slots.indexOf("purple")),
                    spindexer.kick()
            ));
            spindexer.slots.remove(spindexer.states.getSelectedIndex());
            spindexer.slots.add(spindexer.states.getSelectedIndex(), "empty");
        }
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && flywheel.primed && spindexer.slots.contains("green")) {
            actions.schedule(new SequentialAction(
                    spindexer.sortTo(spindexer.slots.indexOf("green")),
                    spindexer.kick()
            ));
        }

        drivetrain.drive(x, y, rx, false);
        drivetrain.periodic();
        spindexer.periodic();
        flywheel.setVls(20);
        flywheel.periodic();
        driver.readButtons();
        actions.run();
        telemetryControl.update();
    }

    public Action intakePeriodic() {
        if (spindexer.isMoving || !spindexer.slots.contains("empty")) {
            return intake.stopIntake();
        } else {
            if (spindexer.states.getSelectedIndex() > 2 && Objects.equals(spindexer.slots.get(spindexer.states.getSelectedIndex() - 3), "empty")) {
                spindexer.slots.remove(spindexer.states.getSelectedIndex() - 3);
                spindexer.slots.add(spindexer.states.getSelectedIndex() - 3, colorSensor.getColour());
            }
            return new SequentialAction(
                    spindexer.intakeToEmpty(),
                    intake.runIntake()
            );
        }
    }
}