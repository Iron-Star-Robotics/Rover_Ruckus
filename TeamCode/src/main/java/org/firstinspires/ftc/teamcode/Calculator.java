package org.firstinspires.ftc.teamcode;

public class Calculator {
    public static double wrapAngle(final double angle1, final double angle2) {
        return (angle1 + angle2) % 360;
    }

    public static double wrapAngleMinus(final double angle1, final double angle2) {
        return 360 - ((angle1 + angle2) % 360);
    }
}
