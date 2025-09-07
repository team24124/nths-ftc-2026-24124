package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

@Config
@TeleOp(name = "Slide Debugger", group = "test")
public class SlideDebugger extends OpMode {
    private DcMotorEx leftSlide, rightSlide;
    private PIDController controller;
    private VoltageSensor voltageSensor;
    private List<LynxModule> hubs;
    public static double p = 0.00945, i = 0, d = 0.0001, f = 0.01;
    public static int target = 0;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        leftSlide = hardwareMap.get(DcMotorEx.class, "left_slide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "right_slide");

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        controller = new PIDController(p, i, d);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        controller.setPID(p, i, d);
        int armPos = leftSlide.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double power = (pid + f) * (12.0 / voltageSensor.getVoltage()); // Compensate for voltage discrepancies

        leftSlide.setPower(power);
        rightSlide.setPower(power);

        telemetry.addData("Position", leftSlide.getCurrentPosition());
        telemetry.addData("Target", target);
    }
}
