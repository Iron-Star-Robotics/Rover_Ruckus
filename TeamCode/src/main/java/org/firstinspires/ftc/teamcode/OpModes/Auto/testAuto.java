package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Config
@Autonomous(name="testauto")
public class testAuto extends LinearOpMode {
    public static double LEFT_MOTOR_POWER = .1;
    public static double RIGHT_MOTOR_POWER = .1;
    Robot robot;
    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.start();

        waitForStart();
        robot.drive.setTargetVelocity(new Pose2d(
                72.0,
                0,
                0
        ));

        idle();
    }
}
