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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Motion.DriveConstants;

public abstract class MecanumDriveBase extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0.5, 0, 0.01);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.03, 0, 0.01);
    // tune this

    private DriveConstraints constraints;
    private TrajectoryFollower follower;
    private Trajectory trajectory;

    public MecanumDriveBase() {
        super(DriveConstants.TRACK_WIDTH);

        constraints = new MecanumConstraints(DriveConstants.BASE_CONSTRAINTS, DriveConstants.TRACK_WIDTH);
        follower = new MecanumPIDVAFollower(this, TRANSLATIONAL_PID, HEADING_PID,
                DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic);
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d currentPose) {
        return new TrajectoryBuilder(currentPose, constraints);
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
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
