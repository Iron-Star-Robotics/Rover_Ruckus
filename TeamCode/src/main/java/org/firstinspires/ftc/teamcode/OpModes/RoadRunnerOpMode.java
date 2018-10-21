package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Motion.PID;
import org.firstinspires.ftc.teamcode.Motion.RobotTankDrive;

public class RoadRunnerOpMode extends LinearOpMode {
    private static final double KP = 1;
    private static final double KD = 1;
    private double error = 0;
    private static final double targetDistance = 20; // in inches
    private static final double tgtAngle = 45;

    @Override
    public void runOpMode() {
        RobotTankDrive drive = new RobotTankDrive(hardwareMap);
        PID pid = new PID(KP, KD, 0);
        PID angle = new PID(KP, KD, 0);
        waitForStart();
        Trajectory trajectory = drive.trajectoryBuilder()
                .turnTo(Math.PI)
                .waitFor(2)
                .turnTo(0)
                .waitFor(2)
                .lineTo(new Vector2d(60, 0))
                .waitFor(2)
                .splineTo(new Pose2d(0, 40, 0))
                .build();


        while (opModeIsActive()) {

        }
    }
}
