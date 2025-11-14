package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp(name = "SetBLUE", group = "mode")
public class SetBLUE extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PoseStorage.currentAlliance = PoseStorage.Alliance.BLUE;
        telemetry.addLine("Alliance successfully set to BLUE");
        telemetry.update();
    }
}
