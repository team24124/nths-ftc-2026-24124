package org.firstinspires.ftc.teamcode.opmode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Reset Encoders", group = "!")
public class Reset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftSlide = hardwareMap.get(DcMotorEx.class, "left_slide");
        DcMotorEx rightSlide = hardwareMap.get(DcMotorEx.class, "right_slide");
        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Encoders successfully reset to zero.");
        telemetry.update();
    }
}
