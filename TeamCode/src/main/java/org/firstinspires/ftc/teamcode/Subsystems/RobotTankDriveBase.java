package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Motion.DriveConstants;

@Config
@Deprecated
public abstract class RobotTankDriveBase extends TankDrive {
    public static PIDCoefficients DISPLACEMENT_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients CROSS_TRACK_PID = new PIDCoefficients(0, 0, 0);


    private DriveConstraints constraints;
    private TankPIDVAFollower follower;
    private Trajectory trajectory;

    public RobotTankDriveBase() {
        super(DriveConstants.TRACK_WIDTH);

        constraints = new TankConstraints(DriveConstants.BASE_CONSTRAINTS, DriveConstants.TRACK_WIDTH);
        follower = new TankPIDVAFollower(this, DISPLACEMENT_PID, CROSS_TRACK_PID,
                DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic);
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    public void robotTurn(double radians) {
        this.followTrajectory(this.trajectoryBuilder().turn(radians).build());
    }

    public void robotLine(Vector2d pos) {
        this.followTrajectory(this.trajectoryBuilder().lineTo(pos).build());
    }

    public void robotMoveForward(int amount) {
        this.followTrajectory(this.trajectoryBuilder().forward(amount).build());
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

    public void setTrajectory(Trajectory trajectory) {
        this.trajectory = trajectory;
    }

    public Trajectory getTrajectory() { return trajectory; }

    public boolean isFollowingTrajectory() {
        return follower.isFollowing();
    }

    public Pose2d getFollowingError() {
        return follower.getLastError();
    }

    public abstract PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode);

    public abstract void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients);
}
