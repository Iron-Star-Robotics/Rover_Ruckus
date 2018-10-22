package org.firstinspires.ftc.teamcode.Motion;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;


public class Trajectory {
    private Path path;
    private MotionProfile axialProfile;

    private PIDFController axialController, lateralController, headingController;
    private PIDCoefficients axial, lateral, heading;
    private double duration;
    private double kV;

    private Trajectory(Path path, PIDCoefficients axial, PIDCoefficients lateral, PIDCoefficients heading, double kV) {
        this.path = path;
        this.axialProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0, 0),
                new MotionState(path.length(), 0, 0, 0),
                1,
                1,
                1
        );
        this.axial = axial;
        this.lateral = lateral;
        this.heading = heading;
        this.kV = kV;

        duration = axialProfile.duration();
        this.axialController = new PIDFController(axial, kV, 0, 0);
        this.lateralController = new PIDFController(lateral, kV, 0, 0);
        this.headingController = new PIDFController(heading, kV, 0, 0);
    }

    public Pose2d update(double time, Pose2d pose) {
        Pose2d targetPose = path.get(axialProfile.get(time).getX());

        Pose2d targetVelocity = path.deriv(axialProfile.get(time).getX()).times(axialProfile.get(time).getV());
        Vector2d trackingError = pose.pos().minus(targetPose.pos()).rotated(targetPose.getHeading());

        Vector2d trackingCorrection = new Vector2d(
                axialController.update(trackingError.getX()),
                lateralController.update(trackingError.getY())
        );

        double headingCorrection = headingController.update(pose.getHeading() - targetPose.getHeading());
        Pose2d correction = new Pose2d(trackingCorrection, headingCorrection);
        return targetVelocity.plus(correction);
    }


}
