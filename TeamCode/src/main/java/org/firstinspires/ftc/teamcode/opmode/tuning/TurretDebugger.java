package org.firstinspires.ftc.teamcode.opmode.tuning;

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

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;

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

    private final double motorTPR = 537.6;

    private boolean align = false;
    private boolean redAlign = false;

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
        pd.setPD(Kp, Kd, a);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        double y = driver.getLeftY();
        double x = driver.getLeftX();
        double rx = driver.getRightX();

        pd.setPD(Kp, Kd, a);

        if (align) {
            int position = turretBase.getCurrentPosition();
            double botY = drivetrain.getPosition().position.y;
            double botX = drivetrain.getPosition().position.x;
            double heading = drivetrain.getHeading();
            double target;

            if (limelight.isDetected()) {
                target = (position - (motorTPR/360) * limelight.degreeOffset()) % motorTPR;
            } else {
                double theta;
                if (redAlign) {
                    theta = Math.atan2(72 - botY, 72 - botX);
                } else {
                    theta = Math.atan2(72 - botY, -72 - botX);
                }
                if (theta < 0) theta += (Math.PI * 2);
                theta = (theta + Math.PI/2) % (Math.PI*2);

                double thetaTicks = (motorTPR / (Math.PI * 2)) * theta;

                double headingTicks = ((heading + Math.PI) % (Math.PI * 2)) * (motorTPR / (Math.PI * 2));

                target = thetaTicks - headingTicks + (motorTPR/2);
                target %= motorTPR;
            }
            if (target < 0) target += motorTPR;

            double power = pd.calculate(position, target, voltageSensor.getVoltage());
            turretBase.setPower(power);
        }

        if (driver.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
            power(-1);
        }
        if (driver.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
            power(1);
        }

        if (driver.wasJustPressed(GamepadKeys.Button.A)) {
            align = true;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            align = false;
        }

        drivetrain.drive(x, y, rx, false);
        drivetrain.periodic();

        driver.readButtons();

        telemetry.addData("Position", turretBase.getCurrentPosition());
        telemetry.addData("Align", align);
        telemetry.addData("Red Align", redAlign);
        telemetry.addData("Power", turretBase.getPower());
        telemetry.update();
    }

    // Set power if within tick range of turret
    public void power(double power) {
        if (!(turretBase.getCurrentPosition() < 0) && !(turretBase.getCurrentPosition() > 537.6)) {
            turretBase.setPower(power);
        }
    }
}