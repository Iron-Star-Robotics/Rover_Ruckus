package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

public class Calculator {
    public static double wrapAngle(final double angle1, final double angle2) {
        return (angle1 + angle2) % 360;
    }

    public static double wrapAngleMinus(final double angle1, final double angle2) {
        return 360 - ((angle1 + angle2) % 360);
    }

    public static double euclideanDistance(final Pose2d pose1, final Pose2d pose2) {
        return Math.sqrt(Math.pow(pose1.getX() - pose2.getX(), 2) + Math.pow(pose1.getY() - pose2.getY(), 2));
    }
}
