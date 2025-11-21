package org.firstinspires.ftc.teamcode.opmode.debug.tune;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpTrajectories;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.TelemetryControl;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.util.plotting.InterpHermiteLUT;
import org.firstinspires.ftc.teamcode.util.plotting.InterpLUT;

import java.util.List;

@Config
@TeleOp(name = "FlyWheel", group = "tuning")
public class FlywheelDebugger extends OpMode {
    private GamepadEx driver;
    private List<LynxModule> hubs;
    private Flywheel flywheel;
    private Drivetrain drivetrain;
    private Spindexer spindexer;
    private ActionScheduler actions;
    private TelemetryControl telemetryControl;
    private TeleOpTrajectories trajectories;
    public static double Kp = 0.0005;
    public static double Kv = 0.00042;
    public static double velocity = 1; // Enter ticks/second
    private boolean debugServo = true;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        actions = ActionScheduler.INSTANCE;
        actions.init();
        driver = new GamepadEx(gamepad1);
        flywheel = new Flywheel(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        telemetryControl = new TelemetryControl(telemetry);
        drivetrain = new RobotCentricDrive(hardwareMap, new Pose2d(0, 0, 0)); // Start robot at the center of the field
        telemetryControl.subscribe(flywheel).subscribe(drivetrain).subscribe(spindexer);
        trajectories = TeleOpTrajectories.INSTANCE;
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        flywheel.setVelPID(Kp, Kv);

        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            debugServo = !debugServo;
        }

        if (debugServo) {
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                flywheel.flap.setPosition(flywheel.flap.getPosition()+0.02);
            }
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                flywheel.flap.setPosition(flywheel.flap.getPosition()-0.02);
            }
        } else {
            if (driver.wasJustPressed(GamepadKeys.Button.A)) {
                actions.schedule(flywheel.runFlywheel());
            }
            if (driver.wasJustPressed(GamepadKeys.Button.B)) {
                actions.schedule(flywheel.stopFlywheel());
            }

            if (Utilities.isBetween(flywheel.wheel1.getVelocity(), velocity - 20, velocity + 50)) {
                if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    actions.schedule(
                            new SequentialAction(
                                spindexer.sortTo(0),
                                spindexer.kick()
                            )
                    );
                }
            }
            if (driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                actions.schedule(spindexer.sortTo(3));
            }

            if (flywheel.powered) {
                flywheel.power(velocity);
            } else {
                flywheel.power(0);
            }
        }


        //
        telemetryControl.getTelemetry().addData("Distance", trajectories.distanceToTarget(drivetrain, true));
        telemetryControl.getTelemetry().addData("Servo debug", debugServo);
        driver.readButtons();
        drivetrain.periodic();
        spindexer.periodic();
        flywheel.adjustFlap(trajectories.distanceToTarget(drivetrain, true));
        actions.run();
        telemetryControl.update();
    }
}
