package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Motion.DriveConstants;
import org.firstinspires.ftc.teamcode.Motion.Paths;

@Autonomous(name="singleSampleCrater")
public class SingleSampleCrater extends RRAuto {
    @Override
    public void runOpMode() {
        init(true); // complete sampling routine
        waitForStart();

        sample();
        Trajectory trajectory = robot.drive.trajectoryBuilder()
                .splineTo(new Pose2d(-60, 63, Math.PI))
                .build();

        robot.drive.setTrajectory(trajectory);
        robot.drive.followTrajectory(trajectory);

        while (opModeIsActive() && robot.drive.isFollowingTrajectory()) {

        }

        trajectory = robot.drive.trajectoryBuilder(DriveConstants.FAST_CONSTRAINTS)
                .waitFor(1)
                .lineTo(new Vector2d(-50, robot.drive.getPoseEstimate().getY()), new ConstantInterpolator(Math.PI))
                .build();


        robot.drive.setTrajectory(trajectory);
        robot.drive.followTrajectory(trajectory);

        while (opModeIsActive() && robot.drive.isFollowingTrajectory()) {

        }

        servo.setPosition(-.75);

        trajectory = robot.drive.trajectoryBuilder(DriveConstants.FAST_CONSTRAINTS)
                .strafeLeft(6)
                .waitFor(1)
                .lineTo(new Vector2d(55, robot.drive.getPoseEstimate().getY()), new ConstantInterpolator(Math.PI))
                .build();

        robot.drive.setTrajectory(trajectory);
        robot.drive.followTrajectory(trajectory);

        while (opModeIsActive() && robot.drive.isFollowingTrajectory()) {

        }
        // deposit marker
    }
}
