package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class GlobalRobotData {
    public static boolean hasAutonRun = false;
    public static int endAutonBallCount = -1;
    public static Pose endAutonPose = null;
    public static double endAutonTurretAngleDegrees = Double.NaN;
    public static double endAutonTurretServoCommandAngleDegrees = Double.NaN;
    public static double endAutonTurretServoOffsetDegrees = Double.NaN;

    public static COLOR allianceSide = COLOR.BLUE;

    // Offseason event setup:
    // RED  = robot starts on the y = 0 end of the 10x6-square field
    // BLUE = robot starts on the y = 240 end of the 10x6-square field
    public static COLOR startingSide = COLOR.BLUE;

    public enum COLOR {
        RED,
        BLUE
    }
}
