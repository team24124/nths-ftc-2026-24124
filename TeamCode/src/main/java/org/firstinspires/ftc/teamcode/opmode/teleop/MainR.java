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
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slides;
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
        // Get all hubs (Control Hub internal + any Expansion Hubs)
        hubs = hardwareMap.getAll(LynxModule.class);

        // Set bulk caching mode
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
        // MANUAL mode: bulk cache refresh happens once per loop
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        double y = -driver.getLeftY();
        double x = driver.getLeftX();
        double rx = -driver.getRightX();

        // Alter drivetrain speeds
        if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.actions.schedule(new InstantAction(robot.driveTrain.getSpeeds()::previous));
        }
        if (driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.actions.schedule(new InstantAction(robot.driveTrain.getSpeeds()::next));
        }

        // Enable AT following
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            alignToAT = true;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            alignToAT = false;
        }

        // Autonomous alignment
        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            robot.actions.schedule(trajectory.vectorAlign(robot.driveTrain.getDrive(), robot.limelight.PSSTargetVectorRobotSpace()));
        }

        // Reset Top Arm to starting positions
        if (operator.wasJustPressed(GamepadKeys.Button.BACK)) {
            robot.actions.schedule(robot.resetControlArm());
        }

        if (operator.wasJustPressed(GamepadKeys.Button.START)) {
            robot.slides.positions.setSelected(Slides.State.HOME);
            robot.slides.stopAndResetEncoders();
        }

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            if(!robot.slides.isMoving){
                robot.actions.schedule(robot.slides.nextPos());
            }
        }
        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            if(!robot.slides.isMoving){
                robot.actions.schedule(new SequentialAction(
                        robot.slides.prevPos(),
                        new InstantAction(
                                () -> {
                                    if (robot.slides.positions.getSelected() == Slides.State.HOME)
                                        robot.slides.stopAndResetEncoders();
                                }
                        )
                ));
            }
        }

        if (operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            if(!robot.slides.isMoving && !robot.isExtended()){
                robot.actions.schedule(robot.collectFromWall());
            }
        }

        // Collection Claw Pivots
        if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.actions.schedule(robot.claw.prevPivot());
        }
        if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.actions.schedule(robot.claw.nextPivot());
        }

        // Periodic functions
        if (!robot.driveTrain.getDrive().isBusy) {
            if (robot.limelight.isDetected() && alignToAT) {
                robot.limelight.setPipeline(Limelight.Pipeline.AT1);
                robot.driveTrain.align(x, robot.limelight.distance(), robot.limelight.degreeOffset());
            } else {
                robot.driveTrain.drive(x, y, rx);
            }
        }


        robot.telemetryControl.update(); // Update telemetry

        driver.readButtons();
        operator.readButtons();

        robot.driveTrain.periodic();
        robot.slides.periodic();
        robot.arm.periodic();

        robot.actions.run(); // Call this in order for scheduled actions to run
    }

    @Override
    public void stop() {
        robot.telemetryControl.unsubscribeAll();
        robot.actions.stop();
    }
}
