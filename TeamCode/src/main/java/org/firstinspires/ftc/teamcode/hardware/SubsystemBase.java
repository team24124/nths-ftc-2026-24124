package org.firstinspires.ftc.teamcode.hardware;

public interface SubsystemBase {
    /**
     * Method called periodically by the Action Scheduler in OpMode
     * Subsystems running PID loops are required to use periodic()
     * Subsystems powering drivetrain are required to use periodic()
     * Subsystems consisting of servos do not need to use periodic()
     * By default has an empty body, able to be overridden.
     */
    default void periodic() {}

    String getName();
}
