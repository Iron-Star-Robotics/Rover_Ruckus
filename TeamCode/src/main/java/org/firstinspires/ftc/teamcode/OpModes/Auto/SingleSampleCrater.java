package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.apache.commons.math3.util.MathUtils;
import org.firstinspires.ftc.teamcode.Motion.DriveConstants;
import org.firstinspires.ftc.teamcode.Motion.Paths;

@Autonomous(name="SingleSampleCrater")
public class SingleSampleCrater extends RRAuto {

    @Override
    public void runOpMode() {
        init(true); // complete sampling routine
        waitForStart();
        sample();
        Pose2d desiredPos;

        //robot.drive.followFullTrajectory(Paths.splineToDepot(robot.drive.getPoseEstimate()));
        /*

        Trajectory trajectory = robot.drive.trajectoryBuilder(new Pose2d(18.566,25.8, Math.toRadians(115)))
                .splineTo(new Pose2d(-60,60, Math.PI))
                .build();
        robot.drive.setTrajectory(trajectory);
        robot.drive.followTrajectory(trajectory);
        while (opModeIsActive() && robot.drive.isFollowingTrajectory());


        /* crater center
        Trajectory trajectory = robot.drive.trajectoryBuilder()
                .turnTo(Math.toRadians(160))
                .forward(10)
                .build();
        robot.drive.setTrajectory(trajectory);
        robot.drive.followTrajectory(trajectory);
        while (opModeIsActive() && robot.drive.isFollowingTrajectory());

       trajectory = robot.drive.trajectoryBuilder()
               .lineTo(new Vector2d(-12, 38), new ConstantInterpolator(Math.toRadians(160)))
               .build();

        robot.drive.setTrajectory(trajectory);
        robot.drive.followTrajectory(trajectory);
        while (opModeIsActive() && robot.drive.isFollowingTrajectory());

        trajectory = robot.drive.trajectoryBuilder()
                .turnTo(5 * Math.PI / 4)
                .build();

        robot.drive.setTrajectory(trajectory);
        robot.drive.followTrajectory(trajectory);
        while (opModeIsActive() && robot.drive.isFollowingTrajectory());

        telemetry.log().add("pose x: " + robot.drive.getPoseEstimate().getX() + " pose get y: " + robot.drive.getPoseEstimate().getY());
        trajectory = robot.drive.trajectoryBuilder()
                .forward(60)
                .build();

        robot.drive.setTrajectory(trajectory);
        robot.drive.followTrajectory(trajectory);
        while (opModeIsActive() && robot.drive.isFollowingTrajectory());
        */

        // deposit marker
        /* depot cetner
        Trajectory trajectory = robot.drive.trajectoryBuilder(new Pose2d(-45.566, 38.8, 3 * Math.PI / 4))
                .lineTo(new Vector2d(-26.566, 18.8), new ConstantInterpolator(3 * Math.PI / 4))
                .waitFor(.5)
                .turnTo(5 * Math.PI / 4)
                .build();

        telemetry.log().add("pose x: " + robot.drive.getPoseEstimate().getX() + " pose y: " + robot.drive.getPoseEstimate().getY());
        sleep(3000);

        */

        if (goldPos == GoldPos.LEFT) {
            robot.drive.followFullTrajectory(Paths.leftCraterSample());
            desiredPos = new Pose2d(18.566, 25.8, Math.toRadians(125));
        }
        else if (goldPos == GoldPos.MIDDLE) {
            robot.drive.followFullTrajectory(Paths.centerCraterSample());
            desiredPos = new Pose2d(18.566, 25.8, Math.toRadians(125));
        }
        else {
            robot.drive.followFullTrajectory(Paths.rightCraterSample());
            desiredPos = new Pose2d(24,17.8, Math.toRadians(125));
        }

        robot.drive.followFullTrajectory(Paths.splineToDepot(desiredPos));


    }

    @Override
    public double getBias() {
        return Math.PI / 4;
    }

    @Override
    public Location getLocation() {
        return Location.CRATER;
    }

    // 20.566 50.8
}
