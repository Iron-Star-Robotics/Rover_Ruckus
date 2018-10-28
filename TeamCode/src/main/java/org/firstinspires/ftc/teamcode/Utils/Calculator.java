package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.motors.RevRobotics40HdHexMotor;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Motion.RobotTankDrive;

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

    public static double Inches2Encoder(double inches){
        return (40 * MotorConfigurationType.getMotorType(RevRobotics40HdHexMotor.class).getTicksPerRev());

    }

    public static double encoderRevolutions(double revolutions) {
        return MotorConfigurationType.getMotorType(RevRobotics40HdHexMotor.class).getTicksPerRev() * revolutions;
    }

    public static double Encoder2Inches(double encoderDistance){
        return (encoderDistance / MotorConfigurationType.getMotorType(RevRobotics40HdHexMotor.class).getTicksPerRev() * 40);
    }




}
