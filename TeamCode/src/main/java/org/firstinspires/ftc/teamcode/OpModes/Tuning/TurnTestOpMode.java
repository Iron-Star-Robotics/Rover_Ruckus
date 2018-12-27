package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.RobotTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.RobotTankDriveBase;
import org.firstinspires.ftc.teamcode.Utils.Misc.DashboardUtil;


/*
 * This is a simple routine to test turning capabilities. If this is consistently overshooting or
 * undershooting by a significant amount, re-run TrackWidthCalibrationOpMode.
 */
@Autonomous(name="turntest")
public class TurnTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        RobotTankDriveBase drive = new RobotTankDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .turnTo(Math.PI / 2)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);
        while (!isStopRequested() && drive.isFollowingTrajectory()) {
            drive.update();
        }
    }
}
