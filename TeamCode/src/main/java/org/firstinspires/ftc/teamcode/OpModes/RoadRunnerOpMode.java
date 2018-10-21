package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
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
        FtcDashboard dashboard = FtcDashboard.getInstance();

        PID pid = new PID(KP, KD, 0);
        PID angle = new PID(KP, KD, 0);
        waitForStart();
        Trajectory trajectory = drive.trajectoryBuilder()
                .lineTo(new Vector2d(60, 0))
                .turnTo(Math.PI)
                .lineTo(new Vector2d(60, 60))
                .turnTo(1.5 * Math.PI)
                .lineTo(new Vector2d(0, 60))
                .turnTo(2 * Math.PI)
                .lineTo(new Vector2d(0,0))
                .build();


        while (opModeIsActive()) {
            
        }
    }
}
