package org.firstinspires.ftc.teamcode.Motion;

/*
    Trajectory Paths to follow

 */

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

public class Paths {
    private static Pose2d BLUE_CRATER_SAMPLE = new Pose2d(20,20, 45);
    private static Pose2d BLUE_DEPOT = new Pose2d(-63, 63, -45);
    public static Pose2d BLUE_CRATER_LANDER = new Pose2d();

    public static Trajectory DEPOT_TO_CRATER_BLUE(Pose2d currentPose) {
        return new TrajectoryBuilder(currentPose, DriveConstants.BASE_CONSTRAINTS)
                .splineTo(BLUE_CRATER_SAMPLE)
                .build();
    }

    public static Trajectory craterToDepot(Pose2d pose) {
        return new TrajectoryBuilder(pose, DriveConstants.BASE_CONSTRAINTS)
                .splineTo(BLUE_DEPOT)
                .build();
    }


}
