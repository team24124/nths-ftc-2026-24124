package org.firstinspires.ftc.teamcode.opmode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Reset Encoders", group = "!")
public class Reset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx x = hardwareMap.get(DcMotorEx.class, "x");
        DcMotorEx y = hardwareMap.get(DcMotorEx.class, "y");
        DcMotorEx z = hardwareMap.get(DcMotorEx.class, "z");

        x.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        y.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        z.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addLine("Encoders successfully reset to zero.");
        telemetry.update();
    }
}
