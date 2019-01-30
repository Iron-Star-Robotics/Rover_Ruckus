package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Motion.Paths;

@Autonomous(name="singleSampleCrater")
public class SingleSampleCrater extends RRAuto {
    @Override
    public void runOpMode() {
        init(true); // complete sampling routine
        waitForStart();

        sample();
        // deposit marker
    }
}
