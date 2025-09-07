package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Claw;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Slides;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;

import java.util.List;

@Config
@TeleOp(name = "Component Debugger", group = "test")
public class ComponentDebugger extends OpMode {
    public static double clawTarget = 0;
    public static double elbowTarget = 0;
    public static int armTarget = 0;
    public static int slideTarget = 0;

    private Arm arm;
    private Claw claw;
    private Slides slides;

    private ActionScheduler actions;

    private GamepadEx gamepad;

    private List<LynxModule> hubs;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
        slides = new Slides(hardwareMap);

        gamepad = new GamepadEx(gamepad1);

        actions = ActionScheduler.INSTANCE;
        actions.init();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        if (gamepad.wasJustPressed(GamepadKeys.Button.A)) actions.schedule(claw.setElbowPosition(elbowTarget));
        if (gamepad.wasJustPressed(GamepadKeys.Button.A)) actions.schedule(claw.setClawPosition(clawTarget));
        if (gamepad.wasJustPressed(GamepadKeys.Button.A)) actions.schedule(slides.moveTo(slideTarget));
        if (gamepad.wasJustPressed(GamepadKeys.Button.A)) actions.schedule(arm.moveTo(armTarget));

        gamepad.readButtons();

        actions.run();

        telemetry.addLine("A to schedule");
        telemetry.addLine();
        telemetry.addLine("B to pause telemetry");
        telemetry.addLine();
        telemetry.addData("\n\nRuntime", "%.2f", getRuntime());
        if (!gamepad.isDown(GamepadKeys.Button.B)) {
            telemetry.update();
        }
    }
}
