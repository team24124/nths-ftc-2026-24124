package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Simple static field serving as a storage medium for the bot's pose
 * This allows different classes/OpModes to set and read from a central source of truth
 * A static field allows data to persist between OpModes
 */
public class PoseStorage {
    public static Pose2d currentPose = new Pose2d(0,0,0);

    public enum Alliance { RED, BLUE }
    public static Alliance currentAlliance = Alliance.BLUE;
    public static List<String> pattern = new ArrayList<>(Arrays.asList("purple", "purple", "green"));
}