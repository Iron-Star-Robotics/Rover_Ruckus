package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Motion.RobotTankDrive;

@TeleOp
public class RRTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotTankDrive tank = new RobotTankDrive(hardwareMap);
        double xMove, yMove = 0;
        waitForStart();
        while (opModeIsActive()) {
            tank.gamepadDrive(gamepad1.right_stick_x, gamepad1.left_stick_y);
        }
    }
}
