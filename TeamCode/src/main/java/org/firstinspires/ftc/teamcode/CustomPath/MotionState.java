package org.firstinspires.ftc.teamcode.CustomPath;

// Represents the kinematic state of a motion profile at time t

public class MotionState {
    private double displacement, velocity, acceleration;

    public MotionState(double displacement, double velocity, double accleration) {
        this.displacement = displacement;
        this.acceleration = accleration;
        this.velocity = velocity;
    }

    // returns the motion state at a specific time
    MotionState get(double time) {
        if (acceleration == 0.0) {
            return new MotionState(
                    displacement + velocity * time,
                    0.0,
                    0.0
            );
        }
        else {
            return new MotionState(
                    displacement + velocity * time + acceleration / 2 * time * time,
                    velocity + acceleration * time,
                    acceleration
            );
        }
    }
}
