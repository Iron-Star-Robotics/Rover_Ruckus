package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Motion.DriveConstants;
import org.firstinspires.ftc.teamcode.Motion.Paths;

@Autonomous(name="singleSampleCrater")
public class SingleSampleCrater extends RRAuto {
    private DcMotorEx liftMotor;
    @Override
    public void runOpMode() {
        init(true); // complete sampling routine
        liftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();


        sample();

        Trajectory trajectory = robot.drive.trajectoryBuilder()
                .waitFor(1)
                .lineTo(new Vector2d(45,17.8), new ConstantInterpolator(0))
                .waitFor(1)
                .lineTo(new Vector2d(24,17.8), new ConstantInterpolator(0))
                .waitFor(1)
                .build();

        robot.drive.setTrajectory(trajectory);
        robot.drive.followTrajectory(trajectory);

        while (opModeIsActive() && robot.drive.isFollowingTrajectory()) {

        }

        trajectory = robot.drive.trajectoryBuilder()
                .turn(Math.PI / 2)
                .build();

        robot.drive.setTrajectory(trajectory);
        robot.drive.followTrajectory(trajectory);

        while (opModeIsActive() && robot.drive.isFollowingTrajectory()) {

        }

        trajectory = robot.drive.trajectoryBuilder()
                .splineTo(new Pose2d(-60, 68, Math.PI))
                .build();

        robot.drive.setTrajectory(trajectory);
        robot.drive.followTrajectory(trajectory);

        while (opModeIsActive() && robot.drive.isFollowingTrajectory()) {

        }



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
