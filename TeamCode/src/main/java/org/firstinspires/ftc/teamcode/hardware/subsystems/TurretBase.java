package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;

public class TurretBase implements SubsystemBase, TelemetryObservable {
    private final DcMotorEx turretBase; // Runs on CCW
    private final VoltageSensor voltageSensor;
    private PIDF pd;
    private double heading;
    private double Tx;
    private boolean isDetected;
    private boolean redAlign;
    private double botX;
    private double botY;

    public TurretBase(HardwareMap hw) {
        voltageSensor = hw.get(VoltageSensor.class, "Control Hub");

        pd = new PIDF();
        pd.setPD(0,0,0, 537.6);

        turretBase = hw.get(DcMotorEx.class, "turret_base");
        turretBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setHeadings(MecanumDrive drivebase, double Tx, boolean isDetected, boolean redAlign) {
        this.heading = drivebase.localizer.getPose().heading.toDouble();
        this.Tx = Tx;
        this.isDetected = isDetected;
        this.botX = drivebase.localizer.getPose().position.x;
        this.botY = drivebase.localizer.getPose().position.y;
        this.redAlign = redAlign;
    }

    @Override
    public void periodic() {
        int position = turretBase.getCurrentPosition();
        double target;
        if (isDetected) {
            target = (turretBase.getCurrentPosition() + (537.6/360) * Tx) % 537.6; // Current position + Range / 360 degrees * offset degrees
        } else {
            double theta;
            if (redAlign) {
                theta = (Math.atan2(72 - botY, 72 - botX) + Math.PI/2) % (Math.PI*2); // Normalize atan2 to 0 at north CCW
            } else {
                theta = (Math.atan2(72 - botY, -72 - botX) + Math.PI/2) % (Math.PI*2); // Top left position
            }

            double thetaEncoders = (537.6 / (Math.PI * 2)) * theta; // Convert to ticks

            target = thetaEncoders - ((537.6 / (Math.PI * 2)) * heading); // Convert heading to ticks
            //  Position derived ticks           - Heading ticks

            if (target > (537.6/2)) { // Wrapper into 180, 0, -180 degrees
                target -= 537.6;
            } else if (target < -(537.6/2)) {
                target += 537.6;
            }
        }

        double power = pd.calculate(position, target, voltageSensor.getVoltage());
        turretBase.setPower(power);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Position", turretBase.getCurrentPosition());
    }

    @Override
    public String getName() {
        return "Turret Base";
    }
}
