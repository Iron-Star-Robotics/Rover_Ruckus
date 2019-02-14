package org.firstinspires.ftc.teamcode.Utils.Misc;

import com.acmerobotics.roadrunner.Pose2d;

public class Smoother {
    private Pose2d lastCommand = new Pose2d(0, 0, 0);
    private long lastTime = 0;
    public static double exponenet = 2;
    private MODE mode = MODE.LINEAR;

    public enum MODE {
        LINEAR,
        EXPONENTIAL
    }

    public Smoother() {

    }

    public void setMode(MODE mode) {
        this.mode = mode;
    }

    public Pose2d transform(Pose2d original) {
        if (lastTime == 0) {
            lastTime = System.currentTimeMillis();
            return lastCommand;
        }

        long now = System.currentTimeMillis();
        long dt = now - lastTime;
        lastTime = now;

        double r = original.pos().norm();
        double omega = original.getHeading();
        double sigOmega = Math.signum(omega);
        omega = Math.abs(omega);

        switch (mode) {
            case LINEAR: break;
            case EXPONENTIAL:
                r = Math.pow(r, exponenet);
                omega = Math.pow(omega, exponenet);
                break;
        }

        Pose2d command = new Pose2d(original.pos().times(r / original.pos().norm()), omega * sigOmega);
        if (r == 0) command = new Pose2d(0, 0, omega * sigOmega);
        return command;
    }
}
