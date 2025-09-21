package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
    private final DcMotorEx turretBase; // Runs on 0 - 360, CCW, 0 = south
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

        // PD setup
        pd = new PIDF();
        pd.setPD(0,0,0, 537.6);

        turretBase = hw.get(DcMotorEx.class, "turret_base");
        turretBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Second periodic function to set necessary values for feedback derivative loop
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
            target = (position - (537.6/360) * Tx) % 537.6; // Current position - offset degrees converted into ticks % 360 to center on target
        } else {
            double theta;
            // Sets theta to the angle to the target based on field centric coordinates. 0 - 360, CCW, 0 = south after theta normalization
            if (redAlign) {
                theta = Math.atan2(72 - botY, 72 - botX);
            } else {
                theta = Math.atan2(72 - botY, -72 - botX);
            }
            if (theta < 0) theta += (Math.PI * 2); // Convert atan2 from 0 --- 180 --- -180 --- 0 into 0 - 360
            theta = (theta + Math.PI/2) % (Math.PI*2); // Covert 0 = east to 0 = south

            // Convert theta into ticks
            double thetaTicks = (537.6 / (Math.PI * 2)) * theta;

            double headingTicks = ((heading + Math.PI) % (Math.PI * 2)) * (537.6 / (Math.PI * 2)); // Heading in ticks from 0 = south CCW

            target = thetaTicks - headingTicks + (537.6/2); // Calc angle between tick values and normalize
            target %= 537.6;
        }
        if (target < 0) target += 537.6; // Add 360 if angle is negative to normalize once again

        // Calculates power with PD inputs
        double power = pd.calculate(position, target, voltageSensor.getVoltage());
        turretBase.setPower(power);
    }

    // moveTo for use in roadrunner autonomous
    public Action moveTo(MecanumDrive drivebase, double Tx, boolean isDetected, boolean redAlign) {
        return (TelemetryPacket packet) -> {
            heading = drivebase.localizer.getPose().heading.toDouble();
            this.Tx = Tx;
            this.isDetected = isDetected;
            botX = drivebase.localizer.getPose().position.x;
            botY = drivebase.localizer.getPose().position.y;
            this.redAlign = redAlign;

            periodic();

            return true;
        };
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
