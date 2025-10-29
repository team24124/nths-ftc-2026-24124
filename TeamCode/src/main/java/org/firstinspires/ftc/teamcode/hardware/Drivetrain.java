package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.hardware.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.ArraySelect;
import org.firstinspires.ftc.teamcode.util.controllers.PIDF;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;

/**
 * Abstract DriveTrain class used to implement different types of drives
 * By default assumes the drive uses the goBILDA Pinpoint computer and mecanum wheels
 * See {@link RobotCentricDrive} & {@link FieldCentricDrive} for drive implementation logic
 * @see <a href="https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html">gm0 Mecanum Drive Guide</a>
 */
public abstract class Drivetrain implements SubsystemBase, TelemetryObservable {
    private final MecanumDrive drivetrain;
    private final ArraySelect<Double> speeds;
    protected PIDF thetaPD = new PIDF();
    protected final VoltageSensor voltageSensor;

    public Drivetrain(HardwareMap hw, Pose2d start) {
        drivetrain = new MecanumDrive(hw, start);
        speeds = new ArraySelect<>(new Double[]{0.5, 1.0});

        thetaPD.setPD(1,1,0);

        voltageSensor = hw.get(VoltageSensor.class, "Control Hub");
    }

    /**
     * Abstract method used to define how the drivetrain drives
     * given values for an x, y, and a turn
     * @param x Amount of x (Ex. left/right on the left joystick)
     * @param y Amount of y (Ex. up/down on the left joystick)
     * @param rx Amount of turn (Ex. left/right on the right joystick) / Vision target offset
     */
    public abstract void drive(double x, double y, double rx, boolean align);

    public MecanumDrive getDrive(){
        return drivetrain;
    }

    public ArraySelect<Double> getSpeeds() {
        return speeds;
    }

    public Pose2d getPosition() {
        return drivetrain.localizer.getPose();
    }

    //but y is -1 left 1 right while it is actually 0 up and increasing both ways to down 1
    //28.5 is max

    public double getHeading() {
        return (drivetrain.localizer.getPose().heading.toDouble() + Math.PI * 2) % (Math.PI*2);
    }

    /**
     * Set the power of each wheel given a power for each.
     * @param leftFront Power to give the left front wheel
     * @param rightFront Power to give the right front wheel
     * @param leftBack Power to give the left back wheel
     * @param rightBack Power to give the right back wheel
     */
    public void setDrivePowers(double leftFront, double rightFront, double leftBack, double rightBack){
        drivetrain.leftFront.setPower(leftFront);
        drivetrain.rightFront.setPower(rightFront);
        drivetrain.leftBack.setPower(leftBack);
        drivetrain.rightBack.setPower(rightBack);
    }

    @Override
    public String getName() {
        return "Drive Train";
    }
}
