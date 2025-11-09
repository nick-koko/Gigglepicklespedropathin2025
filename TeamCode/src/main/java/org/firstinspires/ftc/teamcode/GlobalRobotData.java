package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class GlobalRobotData {
    public static boolean hasAutonRun = false;
    public static int endAutonBallCount = -1;
    public static Pose endAutonPose = null;

    public static COLOR allianceSide = COLOR.RED;
    public enum COLOR {
        RED,
        BLUE
    }
}


