package org.firstinspires.ftc.teamcode.OpModes.Auto.tests;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

/*
 * This is a simple routine to test translational drive capabilities. If this is *consistently*
 * overshooting or undershooting by a significant amount, check the constants in the drive class.
 */
@Autonomous
public class StraightLineTest extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);


        Trajectory trajectory = robot.drive.trajectoryBuilder()
                .forward(30)
                .build();
        robot.drive.setTrajectory(trajectory);

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.followTrajectory(trajectory);
        while (!isStopRequested() && robot.drive.isFollowingTrajectory()) {

        }
    }
}