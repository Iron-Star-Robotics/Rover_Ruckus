package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utils.Misc.DashboardUtil;

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
        robot.start();
        MecanumDrive drive = robot.drive;

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(30)
                .build();
        robot.drive.setTrajectory(trajectory);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);
        while (!isStopRequested() && drive.isFollowingTrajectory()) {

        }
    }
}