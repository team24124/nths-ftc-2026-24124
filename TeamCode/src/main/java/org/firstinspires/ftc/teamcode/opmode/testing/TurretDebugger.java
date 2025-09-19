package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.util.PIDF;

import java.util.List;

@Config
@TeleOp(name="Turret Debugger", group="test")
public class TurretDebugger extends OpMode {
    private DcMotorEx turretBase;
    private VoltageSensor voltageSensor;
    private Limelight limelight;
    private Drivetrain drivetrain;
    private List<LynxModule> hubs;
    private GamepadEx driver;
    private PIDF pd;

    // --- tune ---
    public static double Kp = 0;
    public static double Kd = 0;
    public static double a = 0;

    // Internal math
    private double target = 0;
    private final double motorTPR = 537.6;

    private boolean align = false;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        driver = new GamepadEx(gamepad1);

        drivetrain = new RobotCentricDrive(hardwareMap, new Pose2d(0,0,0));

        limelight = new Limelight(hardwareMap);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        turretBase = hardwareMap.get(DcMotorEx.class, "turret_base");
        turretBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pd = new PIDF();
        pd.setPD(Kp, Kd, a, 537.6);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        double y = Math.abs(-driver.getLeftY()) > 0.05 ? -driver.getLeftY() : 0;
        double x = Math.abs(driver.getLeftX()) > 0.05 ? driver.getLeftX() : 0;
        double rx = Math.abs(-driver.getRightX()) > 0.05 ? -driver.getRightX() : 0;

        pd.setPD(Kp, Kd, a, 537.6);

        if (align) {
            double botX = drivetrain.getPosition().position.x;
            double botY = drivetrain.getPosition().position.y;
            double Tx = limelight.degreeOffset();
            double heading = drivetrain.getHeading();

            if (limelight.isDetected()) {
                target = (turretBase.getCurrentPosition() + (motorTPR/360) * Tx) % motorTPR; // current position + Range / 360 degrees * offset degrees
            } else {
                double theta = -Math.atan2(72 - botX, 72 - botY) + Math.PI * 2; // -atan2 + 360 to reverse CCW, inverse x & y to transfer 0+ from east to north

                double thetaEncoders = (motorTPR / (Math.PI * 2)) * theta;

                target = ((motorTPR / (Math.PI * 2)) * ((Math.PI * 2) - heading) + thetaEncoders);
                //                TPR per radian           * radians to get T     + more rotation

                if (target > (motorTPR/2)) { // Wrapper into -180, 0, 180 degrees
                    target -= motorTPR;
                } else if (target < -(motorTPR/2)) {
                    target += motorTPR;
                }
            }

            turretBase.setPower(pd.calculate(turretBase.getCurrentPosition(), target, voltageSensor.getVoltage()));
        } else {
            turretBase.setPower(driver.getLeftX());
        }

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            align = true;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            align = false;
        }

        driver.readButtons();
        drivetrain.periodic();

        if (!drivetrain.getDrive().isBusy) {
            drivetrain.drive(x, y, rx, false);
        }
    }
}