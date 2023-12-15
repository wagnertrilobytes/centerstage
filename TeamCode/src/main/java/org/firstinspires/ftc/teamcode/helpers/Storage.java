package org.firstinspires.ftc.teamcode.helpers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class Storage {
    public static int PLANE_SHOOT = 0;
    public static int PLANE_ARM = 0;
    public static int MINIMUM_SLIDE_PLACE_L = -4440;
    public static int MINIMUM_SLIDE_PLACE_R = 4440;
    public static Pose2d currentPose;
}
