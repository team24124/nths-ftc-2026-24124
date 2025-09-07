package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;

import java.util.List;

@TeleOp(name = "MainR", group = "!")
public class MainR extends OpMode {
    private Robot robot;
    private GamepadEx driver, operator;
    private TeleOpTrajectories trajectory;
    private List<LynxModule> hubs;
    private boolean alignToAT = false;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robot = new Robot(hardwareMap, telemetry, true);
        robot.actions = ActionScheduler.INSTANCE;
        trajectory = TeleOpTrajectories.INSTANCE;
        robot.actions.init();
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        // Driver inputs
        double y = -driver.getLeftY();
        double x = driver.getLeftX();
        double rx = -driver.getRightX();

        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.actions.schedule(new InstantAction(robot.driveTrain.getSpeeds()::previous));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.actions.schedule(new InstantAction(robot.driveTrain.getSpeeds()::next));
        }

        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            alignToAT = true;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            alignToAT = false;
        }

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            robot.actions.schedule(trajectory.vectorAlign(robot.driveTrain.getDrive(), robot.limelight.PSSTargetVectorRobotSpace()));
        }

        // Operator inputs


        // Periodic calls
        if (!robot.driveTrain.getDrive().isBusy) {
            if (robot.limelight.isDetected() && alignToAT) {
                robot.limelight.setPipeline(Limelight.Pipeline.AT1);
                robot.driveTrain.align(x, robot.limelight.distance(), robot.limelight.degreeOffset());
            } else {
                robot.driveTrain.drive(x, y, rx);
            }
        }


        robot.telemetryControl.update();

        driver.readButtons();
        operator.readButtons();

        robot.driveTrain.periodic();

        robot.actions.run();
    }

    @Override
    public void stop() {
        robot.telemetryControl.unsubscribeAll();
        robot.actions.stop();
    }
}
