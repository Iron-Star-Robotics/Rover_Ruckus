package org.firstinspires.ftc.teamcode.OpModes.Auto;

import android.graphics.Path;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Motion.Paths;

@Autonomous(name="SingleSampleDepot")
public class SingleSampleDepot extends RRAuto {

    private Pose2d desiredPos;

    @Override
    public void runOpMode() {
        init(true);
        waitForStart();
        sample();

        if (goldPos == GoldPos.LEFT)
            robot.drive.followFullTrajectory(Paths.leftDepotSample());
        else if (goldPos == GoldPos.MIDDLE)
            robot.drive.followFullTrajectory(Paths.centerDepotSample());
        else
            robot.drive.followFullTrajectory(Paths.rightDepotSample());

        Trajectory trajectory = robot.drive.trajectoryBuilder()
                .turnTo(4 * Math.PI / 3)
                .build();
        robot.drive.followFullTrajectory(trajectory);
    }

    @Override
    public double getBias() {
        return 3 * Math.PI / 4;
    }

    @Override
    public Location getLocation() {
        return Location.DEPOT;
    }



}
