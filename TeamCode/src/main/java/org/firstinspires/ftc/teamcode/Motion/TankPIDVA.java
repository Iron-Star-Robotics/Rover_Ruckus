package org.firstinspires.ftc.teamcode.Motion;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.Kinematics;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.util.NanoClock;

public class TankPIDVA extends TrajectoryFollower {

    private PIDFController displacementController;
    private PIDFController crossTrackController;
    private PIDCoefficients displacementCoeffs, crossTrackCoeffs;
    private double kV;
    private TankDrive drive;

    public TankPIDVA(TankDrive drive, PIDCoefficients displacementCoeffs, PIDCoefficients crossTrackCoeffs, double kV, double kA, double kStatic) {
        super(NanoClock.system());
        this.displacementCoeffs = displacementCoeffs;
        this.crossTrackCoeffs = crossTrackCoeffs;

        this.displacementController = new PIDFController(displacementCoeffs);
        this.crossTrackController = new PIDFController(crossTrackCoeffs);
        this.kV = kV;

        this.drive = drive;
    }

    @Override
    public void update(Pose2d currentPose) {
        if (!isFollowing()) { drive.setMotorPowers(0.0, 0.0);  }
        double t = elapsedTime();

        Pose2d targetPose = getTrajectory().get(t);
        Pose2d targetPoseVelocity = getTrajectory().velocity(t);

        Pose2d targetPoseAcceleration = getTrajectory().acceleration(t);

        Pose2d targetRobotPose = Kinematics.fieldToRobotPose(targetPose);
        Pose2d targetRobotPoseVelocity = Kinematics.fieldToRobotPoseVelocity(targetPose, targetPoseVelocity);
        Pose2d targetRobotAcceleration = Kinematics.fieldToRobotPoseAcceleration(targetPose, targetPoseVelocity, targetPoseAcceleration);

        Pose2d currentRobotPose = new Pose2d(currentPose.pos().rotated(-targetPose.getHeading()), currentPose.getHeading() - targetPose.getHeading());
        

    }


}
