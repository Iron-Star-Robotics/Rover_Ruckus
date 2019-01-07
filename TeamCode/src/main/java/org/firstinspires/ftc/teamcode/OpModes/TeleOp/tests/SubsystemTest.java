package org.firstinspires.ftc.teamcode.OpModes.TeleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

public class SubsystemTest extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.start();

        while (opModeIsActive()) {
            if (gamepad1.a)
                robot.lift.raise();
            else if (gamepad1.b)
                robot.lift.lower();
        }
    }
}
