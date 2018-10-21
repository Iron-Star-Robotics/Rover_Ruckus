package org.firstinspires.ftc.teamcode.Motion;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

public class PID {
    private final double KI, KD, KP;
    private Pose2d posTarget;
    private double targetAngle;
    double currTime = 0;
    private static final double MAX_MOTOR_POWER = 0.7;
    private static final double MIN_MOTOR_POWER = 0.1;

    public PID(final double KP, final double KD, final double KI) {
        this.KI = KI;
        this.KD = KD;
        this.KP = KP;
    }

    public void setPosTarget(final Pose2d target) {
        this.posTarget = target;
    }

    public void setAngleTarget(final double angle) {
        this.targetAngle = angle;
    }

    private double euclideanDistance(final Pose2d pose1, final Pose2d pose2) {
        return Math.sqrt(Math.pow(pose1.getX() - pose2.getX(), 2) + Math.pow(pose1.getY() - pose2.getY(), 2));
    }

    public double calculateError(final Pose2d pose) { return euclideanDistance(posTarget, pose); }

    public double getOutput(double runTime, final double prevError, final Pose2d currPos) {
        double error = euclideanDistance(posTarget, currPos);
        double output = KP * error + KD * (error - prevError) / runTime;
        if (output > MAX_MOTOR_POWER) return MAX_MOTOR_POWER;
        else if (output < MIN_MOTOR_POWER)return MIN_MOTOR_POWER;
        return output;
    }

    //public double getAnlgeOuput(double runTime, final double prevError, final double currAngle) {

    //}

}
