package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpTrajectories;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.List;

@Config
@TeleOp(name = "FlyWheel Debugger", group = "tuning")
public class FlywheelDebugger extends OpMode {
    private GamepadEx driver;
    private List<LynxModule> hubs;
    private Flywheel flywheel;
    private TeleOpTrajectories trajectories;
    private Drivetrain drivetrain;
    public static double Kp, Kd, Ks = 0;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        driver = new GamepadEx(gamepad1);
        flywheel = new Flywheel(hardwareMap);

        trajectories = TeleOpTrajectories.INSTANCE;
        drivetrain = new FieldCentricDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), 0));
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        flywheel.setVelPD(Kp, Kd, Ks);

        if (driver.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            flywheel.adjustFlap(trajectories.distanceToTarget(drivetrain, true));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            flywheel.runFlywheel();
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            flywheel.stopFlywheel();
        }

        flywheel.setVls(trajectories.distanceToTarget(drivetrain, true));
        flywheel.periodic();
        drivetrain.periodic();
        driver.readButtons();
    }
}
