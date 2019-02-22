package org.firstinspires.ftc.teamcode.lib.localization.math;

import org.firstinspires.ftc.teamcode.lib.localization.Pose2d;
import org.firstinspires.ftc.teamcode.lib.localization.Vector2d;

import java.util.List;

public class MecanumKinematics {

    Pose2d wheelToRobotVelocity(List<Double> wheelVelocities, double trackWidth, double wheelBase) {
        double k = (trackWidth + wheelBase) / 2.0;
        double sum = 0;
        for (double v : wheelVelocities)
            sum += v;
        return new Pose2d(
                new Vector2d(sum, wheelVelocities.get(1) + wheelVelocities.get(3) - wheelVelocities.get(0) - wheelVelocities.get(2)),
                (wheelVelocities.get(2) + wheelVelocities.get(3) - wheelVelocities.get(0) - wheelVelocities.get(1)) / k
        );

    }


}
