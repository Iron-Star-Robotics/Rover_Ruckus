/*package org.firstinspires.ftc.teamcode.Motion;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Utils.Calculator;


public class PID {
    private final double KI, KD, KP;
    private Pose2d posTarget;
    private double targetHeading;
    private double prevError;
    private double prevTime = 0;
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

    public double getOutput(final double runTime, final Pose2d currPos) {
        final double error = Calculator.euclideanDistance(posTarget, currPos);
        final double output = KP * error + KD * (error - prevError) / (runTime - prevTime);
        if (output > MAX_MOTOR_POWER) return MAX_MOTOR_POWER;
        else if (output < MIN_MOTOR_POWER)return MIN_MOTOR_POWER;
        prevError = error;
        prevTime = runTime;
        return output;
    }

    public double getAngleOutput(final double runTime, final double currHeading) {
        final double error = targetHeading - currHeading;
        final double output =  KP * error + KD * (error - prevError) / (runTime - prevTime);
        prevError = error;
        prevTime = runTime;
        return output;
    }




}
*/

// Not being used atm. Instead we are using Road Runners PID controller