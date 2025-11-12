package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Spindexer;

@TeleOp(name = "Reset", group = "mode")
public class Reset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Spindexer spindexer = new Spindexer(hardwareMap);

        spindexer.stopAndResetEncoders();

        telemetry.addLine("Encoders successfully reset to zero.");
        telemetry.update();
    }
}
