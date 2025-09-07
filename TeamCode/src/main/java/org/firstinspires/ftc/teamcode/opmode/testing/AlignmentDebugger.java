package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.util.PIDF;

import java.util.List;

@Config
@TeleOp(name = "AlignmentDebugger", group = "test")
public class AlignmentDebugger extends OpMode {
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double a = 0;
    public static double integralSumLimit = 0;

    public static double tKp = 0;
    public static double tKi = 0;
    public static double tKd = 0;
    public static double ta = 0;
    public static double tintegralSumLimit = 0;
    
    private Drivetrain driveTrain;
    private Limelight limelight;
    private GamepadEx driver;
    private boolean faceAT = false;
    private boolean faceAndMoveToAT = false;
    private PIDF thetaPID = new PIDF();
    private PIDF distancePID = new PIDF();
    private List<LynxModule> hubs;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        driveTrain = new RobotCentricDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)));
        driver = new GamepadEx(gamepad1);
        limelight = new Limelight(hardwareMap);
        distancePID.setPID(Kp, Ki, Kd, a, integralSumLimit);
        thetaPID.setPID(tKp, tKi, tKd, ta, tintegralSumLimit);
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        distancePID.setPID(Kp, Ki, Kd, a, integralSumLimit);
        thetaPID.setPID(tKp, tKi, tKd, ta, tintegralSumLimit);
        
        double y = -driver.getLeftY();
        double x = driver.getLeftX();
        double rx = -driver.getRightX();

        if (driver.wasJustPressed(GamepadKeys.Button.B)) {
            faceAT = true;
            faceAndMoveToAT = false;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.X)) {
            faceAT = false;
            faceAndMoveToAT = true;
        }
        if (driver.wasJustPressed(GamepadKeys.Button.Y)) {
            faceAT = false;
            faceAndMoveToAT = false;
        }

        if (limelight.isDetected()) {
            if (!driveTrain.getDrive().isBusy && !faceAndMoveToAT) {
                if (faceAT) {
                    limelight.setPipeline(Limelight.Pipeline.AT0);
                    driveTrain.align(x, y, thetaPID.calculate(limelight.degreeOffset(), 0, 8));
                }
            } else if (!driveTrain.getDrive().isBusy && faceAndMoveToAT) {
                limelight.setPipeline(Limelight.Pipeline.AT1);
                driveTrain.align(x, distancePID.calculate(limelight.distance(), 0, 9859594), thetaPID.calculate(limelight.degreeOffset(), 0, 8));
            }
        } else {
            driveTrain.drive(x, y, rx);
            faceAT = false;
            faceAndMoveToAT = false;
        }
    }
}