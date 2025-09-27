package org.firstinspires.ftc.teamcode.opmode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretBase;

@TeleOp(name = "Reset Encoders", group = "!")
public class Reset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Spindexer spindexer = new Spindexer(hardwareMap);
        TurretBase turret = new TurretBase(hardwareMap);

        spindexer.stopAndResetEncoders();
        turret.stopAndResetEncoders();

        telemetry.addLine("Encoders successfully reset to zero.");
        telemetry.update();
    }
}
