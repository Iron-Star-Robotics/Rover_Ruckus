package org.firstinspires.ftc.teamcode.OpModes.Auto.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Motion.Paths;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utils.Hardware.Launcher;

@Config
@Autonomous(name="splinetest")
public class SplineTest extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.start();

        Trajectory trajectory = robot.drive.trajectoryBuilder(new Pose2d(-15, 15,   135))
                .splineTo(new Pose2d(15, 70, 0))
                .build();

        robot.drive.setTrajectory(trajectory);
        telemetry.log().add("pose x: " + robot.drive.getPoseEstimate().getX() + " pose y: " + robot.drive.getPoseEstimate().getX());
        waitForStart();
        robot.drive.followTrajectory(trajectory);
        while(opModeIsActive()) {
            telemetry.log().add("Following: " + robot.drive.isFollowingTrajectory());
            telemetry.log().add(robot.drive.getCurrHeading() + "");
        }


    }

}
