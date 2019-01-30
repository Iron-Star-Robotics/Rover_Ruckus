package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Motion.DriveConstants;

public abstract class MecanumDriveBase extends MecanumDrive {
    //public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0.035, 0, 0.03);
    //public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.43, 0, 0.001);
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(.035,0,0.001);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.43,0,0.01); // gonna try with open loop
    // public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0.53, 0, 0.015);
    // public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.03, 0, 0.01);
    private DriveConstraints constraints;
    private TrajectoryFollower follower;
    private Trajectory trajectory;

    public MecanumDriveBase() {
        super(DriveConstants.TRACK_WIDTH);

        constraints = new MecanumConstraints(DriveConstants.BASE_CONSTRAINTS, DriveConstants.TRACK_WIDTH);
        follower = new MecanumPIDVAFollower(this, TRANSLATIONAL_PID, HEADING_PID, DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, new Pose2d(.5,.5,Math.toRadians(1)));


    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d currentPose) {
        return new TrajectoryBuilder(currentPose, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(DriveConstraints constraints) {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        setPoseEstimate(trajectory.get(0));
    }


    public void updateFollower() {
        follower.update(getPoseEstimate());
    }

    public void update() {
        updatePoseEstimate();
        updateFollower();
    }

    public boolean isFollowingTrajectory() {
        return follower.isFollowing();
    }

    public Pose2d getFollowingError() {
        return follower.getLastError();
    }

    public void setTrajectory(Trajectory trajectory) {
        this.trajectory = trajectory;
    }

    public Trajectory getTrajectory() { return trajectory; }

    public abstract PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode);

    public abstract void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients);

}
