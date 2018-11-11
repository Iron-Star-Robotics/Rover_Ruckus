package org.firstinspires.ftc.teamcode.Motion;

import com.acmerobotics.roadrunner.Vector2d;

public class Pose {
    public double xPos, yPos, heading;

    private Vector2d vecPos;

    public Pose() {}

    public Vector2d getVecPos() {
        return vecPos;
    }

    private void setVecPos() {
        vecPos = new Vector2d(xPos, yPos).rotated(heading);
    }


}
