package org.firstinspires.ftc.teamcode.Motion;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Calculator;


public class PID {
    private final double KI, KD, KP;
    private Pose2d posTarget;
    private double targetHeading;
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

    public void setHeadingTarget(final double heading) {
        this.targetHeading = heading;
    }

    public double getOutput(double runTime, final double prevError, final Pose2d currPos) {
        double error = Calculator.euclideanDistance(posTarget, currPos);
        double output = KP * error + KD * (error - prevError) / runTime;
        if (output > MAX_MOTOR_POWER) return MAX_MOTOR_POWER;
        else if (output < MIN_MOTOR_POWER)return MIN_MOTOR_POWER;
        return output;
    }

    public double getHeadingError(final double currentHeading) {
        return targetHeading - currentHeading;
    }


}
