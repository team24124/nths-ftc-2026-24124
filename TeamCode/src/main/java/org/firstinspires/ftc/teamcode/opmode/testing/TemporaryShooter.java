package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.opmode.teleop.TeleOpTrajectories;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;

import java.util.List;

@TeleOp(name = "temp", group = "test")
public class TemporaryShooter extends OpMode {
    private GamepadEx driver;
    private List<LynxModule> hubs;

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    @Override
    public void init() {
        // Get all hubs (Control Hub internal + any Expansion Hubs)
        hubs = hardwareMap.getAll(LynxModule.class);

        // Set bulk caching mode MANUAl
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        driver = new GamepadEx(gamepad1);
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        // MANUAL mode: bulk cache refresh happens once per loop
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        driver.readButtons();

        if (driver.getButton(GamepadKeys.Button.A)) {
            shooter1.setPower(-1);
            shooter2.setPower(-1);
        }
        if (driver.getButton(GamepadKeys.Button.B)) {
            shooter1.setPower(-0.9);
            shooter2.setPower(-0.9);
        }
        if (driver.getButton(GamepadKeys.Button.Y)) {
            shooter1.setPower(-0.8);
            shooter2.setPower(-0.8);
        }
        if (driver.getButton(GamepadKeys.Button.X)) {
            shooter1.setPower(-0.7);
            shooter2.setPower(-0.7);
        }
        if (driver.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            shooter1.setPower(-0.6);
            shooter2.setPower(-0.6);
        }
        if (driver.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            shooter1.setPower(-0.5);
            shooter2.setPower(-0.5);
        }
        if (driver.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            shooter1.setPower(-0.4);
            shooter2.setPower(-0.4);
        }
        if (driver.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }
    }
}
