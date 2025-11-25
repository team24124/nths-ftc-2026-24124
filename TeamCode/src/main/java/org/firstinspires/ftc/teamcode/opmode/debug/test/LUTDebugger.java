package org.firstinspires.ftc.teamcode.opmode.debug.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpTrajectories;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.TelemetryControl;
import org.firstinspires.ftc.teamcode.util.Utilities;
import org.firstinspires.ftc.teamcode.util.plotting.InterpLUT;

import java.util.List;

@Config
@TeleOp(name = "LUT", group = "tuning")
public class LUTDebugger extends OpMode {
    private List<LynxModule> hubs;
    private TelemetryControl telemetryControl;
    public static double XValue = 0;
    double[] dists = {36, 50, 60, 70, 80, 90, 100, 150}; // Inches
    double[] vels = {1050, 1120, 1200, 1220, 1227, 1233, 1235, 1330}; // Ticks/second
    private InterpLUT lut = new InterpLUT(dists, vels);

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetryControl = new TelemetryControl(telemetry);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        if (gamepad1.dpadUpWasPressed()) {
            XValue += 10;
        }
        if (gamepad1.dpadDownWasPressed()) {
            XValue -= 10;
        }

        telemetryControl.getTelemetry().addData("LUT Y Value", lut.get(XValue));
        telemetryControl.update();
    }
}
