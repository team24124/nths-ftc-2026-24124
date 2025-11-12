package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp(name = "SetRED", group = "mode")
public class SetRED extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PoseStorage.currentAlliance = PoseStorage.Alliance.RED;
        telemetry.addLine("Alliance successfully set to RED");
        telemetry.update();
    }
}
