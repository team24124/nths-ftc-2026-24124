//package org.firstinspires.ftc.teamcode.opmode.testing;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.util.ActionScheduler;
//
//import java.util.List;
//
//@Config
//@TeleOp(name = "Component Debugger", group = "test")
//public class ComponentDebugger extends OpMode {
//    public static double xT = 0;
//    public static double yT = 0;
//    public static int zT = 0;
//
//    private x arm;
//    private y claw;
//    private z slides;
//
//    private ActionScheduler actions;
//
//    private GamepadEx gamepad;
//
//    private List<LynxModule> hubs;
//
//    @Override
//    public void init() {
//        hubs = hardwareMap.getAll(LynxModule.class);
//
//        for (LynxModule hub : hubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }
//
//        x = new x(hardwareMap);
//        y = new y(hardwareMap);
//        z = new z(hardwareMap);
//
//        gamepad = new GamepadEx(gamepad1);
//
//        actions = ActionScheduler.INSTANCE;
//        actions.init();
//    }
//
//    @Override
//    public void loop() {
//        for (LynxModule hub : hubs) {
//            hub.clearBulkCache();
//        }
//
//        if (gamepad.wasJustPressed(GamepadKeys.Button.A)) actions.schedule(x.setPos(xT));
//        if (gamepad.wasJustPressed(GamepadKeys.Button.A)) actions.schedule(y.setPos(yT));
//        if (gamepad.wasJustPressed(GamepadKeys.Button.A)) actions.schedule(z.moveTo(zT));
//
//        gamepad.readButtons();
//
//        actions.run();
//
//        telemetry.addLine("A to schedule");
//        telemetry.addLine();
//        telemetry.addLine("B to pause telemetry");
//        telemetry.addLine();
//        telemetry.addData("\n\nRuntime", "%.2f", getRuntime());
//        if (!gamepad.isDown(GamepadKeys.Button.B)) {
//            telemetry.update();
//        }
//    }
//}
