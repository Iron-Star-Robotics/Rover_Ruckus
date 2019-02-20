package org.firstinspires.ftc.teamcode.Motion;

/*
    Trajectory Paths to follow

 */

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import java.util.ArrayList;
import java.util.List;

public class Paths {
    private static Pose2d DEPOT = new Pose2d(-60, 60, Math.PI);
    public static Pose2d CRATER_LANDER = new Pose2d(14.183, 14.183, Math.PI / 4);


    public static Trajectory unhookLander() {
        return new TrajectoryBuilder(CRATER_LANDER, DriveConstants.BASE_CONSTRAINTS)
                .strafeTo(new Vector2d(10.566,17.8))
                .build();
    }
    public static Trajectory unhookDepot() {
        return new TrajectoryBuilder(new Pose2d(-14.183, 14.183, 3 * Math.PI / 4), DriveConstants.BASE_CONSTRAINTS)
                .strafeTo(new Vector2d(-18.566, 10.8))
                .build();
    }

    public static Trajectory rightCraterSample() {
        return new TrajectoryBuilder(new Pose2d(10.566, 17.8, Math.PI / 4), DriveConstants.BASE_CONSTRAINTS)
                .turnTo(0)
                .waitFor(1)
                .lineTo(new Vector2d(45,17.8), new ConstantInterpolator(0))
                .waitFor(1)
                .lineTo(new Vector2d(24,17.8), new ConstantInterpolator(0))
                .waitFor(1)
                .turnTo(Math.toRadians(125))
                .build();
    }

    public static Trajectory centerCraterSample() {
        return new TrajectoryBuilder(new Pose2d(10.566, 17.8, Math.PI / 4), DriveConstants.BASE_CONSTRAINTS)
                .lineTo(new Vector2d(30.566, 37.8), new ConstantInterpolator(Math.PI / 4))
                .waitFor(1)
                .lineTo(new Vector2d(18.566, 25.8), new ConstantInterpolator(Math.PI / 4))
                .turnTo(Math.toRadians(125))
                .build();

    }

    public static Trajectory leftCraterSample() {
        return new TrajectoryBuilder(new Pose2d(10.566, 17.8, Math.PI / 4), DriveConstants.BASE_CONSTRAINTS)
                .turnTo(Math.toRadians(80.3))
                .waitFor(1)
                .lineTo(new Vector2d(22, 42), new ConstantInterpolator(Math.toRadians(80.3)))
                .waitFor(1)
                .lineTo(new Vector2d(18.566, 25.8), new ConstantInterpolator(Math.toRadians(70.3)))
                .turnTo(Math.toRadians(125))
                .build();
    }

    public static Trajectory rightDepotSample() {
        return new TrajectoryBuilder(new Pose2d(-18.566, 10.8, 3 * Math.PI / 4), DriveConstants.BASE_CONSTRAINTS)
                .turnTo(Math.PI / 2)
                .waitFor(1)
                .lineTo(new Vector2d(-18.566, 42.8), new ConstantInterpolator(Math.PI / 2))
                .waitFor(1)
                .build();
    }


    public static Trajectory splineToDepot(Pose2d pose) {
        return new TrajectoryBuilder(pose, DriveConstants.BASE_CONSTRAINTS)
                .splineTo(DEPOT)
                .build();
    }

    public static Trajectory centerDepotSample() {
        return new TrajectoryBuilder(new Pose2d(-18.566, 10.8, 3 * Math.PI / 4), DriveConstants.BASE_CONSTRAINTS)
                .lineTo(new Vector2d(-45.566, 38.8), new ConstantInterpolator(3 * Math.PI / 4))
                .build();
    }

    public static Trajectory leftDepotSample() {
        return new TrajectoryBuilder(new Pose2d(-18.566, 10.8, 3 * Math.PI / 4), DriveConstants.BASE_CONSTRAINTS)
                .turnTo(Math.toRadians(160.3))
                .lineTo(new Vector2d(-42.8,22), new ConstantInterpolator(Math.toRadians(160.3)))
                .waitFor(1)
                .lineTo(new Vector2d(-29, 22), new ConstantInterpolator(Math.toRadians(160.3)))
                .build();
    }



    public static Trajectory depotToCrater(Pose2d pose) {
        return new TrajectoryBuilder(pose, DriveConstants.BASE_CONSTRAINTS)
                .strafeLeft(6)
                .waitFor(1)
                .lineTo(new Vector2d(55, 63), new ConstantInterpolator(Math.PI))
                .build();
    }

}
