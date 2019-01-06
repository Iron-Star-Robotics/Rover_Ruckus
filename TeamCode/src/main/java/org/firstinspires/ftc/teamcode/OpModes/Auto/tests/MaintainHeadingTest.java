package org.firstinspires.ftc.teamcode.OpModes.Auto.tests;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utils.Hardware.Launcher;

@Autonomous(name="maintainHeading")
public class MaintainHeadingTest extends LinearOpMode{

    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.start();
        MecanumDrive drive = robot.drive;

        Trajectory trajectory = drive.trajectoryBuilder()
                .lineTo(new Vector2d(60,0))
                .build();
        robot.drive.setTrajectory(trajectory);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);
        while (!isStopRequested() && drive.isFollowingTrajectory()) {

        }
    }
}

