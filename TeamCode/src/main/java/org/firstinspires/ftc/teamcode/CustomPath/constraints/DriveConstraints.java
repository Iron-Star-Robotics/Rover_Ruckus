package org.firstinspires.ftc.teamcode.CustomPath.constraints;

import org.firstinspires.ftc.teamcode.Motion.DriveConstants;

public abstract class DriveConstraints {
    double maxVelo, maxAcc, maxAngVel, maxAngAcc;
    public DriveConstraints(double maxVelo, double maxAcc, double maxAngVel, double maxAngAcc) {
        this.maxVelo = maxVelo;
        this.maxAcc = maxAcc;
        this.maxAngVel = maxAngVel;
        this.maxAngAcc = maxAngAcc;
    }
    public abstract double getMaxVelocity();
    public abstract double getMaxAcceleration();
}
