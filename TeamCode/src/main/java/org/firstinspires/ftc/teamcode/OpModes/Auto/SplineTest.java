package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Config
@Autonomous(name="splinetest")
public class SplineTest extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.start();

        Trajectory trajectory = robot.drive.trajectoryBuilder()
                .splineTo(new Pose2d(30, 30, 0))
                .waitFor(1)
                .splineTo(new Pose2d(0, 0, 0))
                .build();

        robot.drive.setTrajectory(trajectory);
        waitForStart();
        robot.drive.followTrajectory(trajectory);
        while(opModeIsActive()) {
            telemetry.log().add("Following: " + robot.drive.isFollowingTrajectory());
            telemetry.log().add(robot.drive.getCurrHeading() + "");
        }
    }
}
