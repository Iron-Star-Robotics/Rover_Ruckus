package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@TeleOp(name="RRTeleop")
@Config

public class RRTeleop extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.start();
        waitForStart();
        while (opModeIsActive()) {
            /*
            robot.drive.setRVelocity(new Pose2d(
                    -(gamepad1.left_stick_x * gamepad1.left_stick_x),
                    -(gamepad1.left_stick_y * gamepad1.left_stick_y),
                    -(gamepad1.right_stick_x * gamepad1.right_stick_x)
            ));*/

            robot.drive.setTargetVelocity(new Pose2d(
                    gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            )); // this might seem weird but actually x is axial and y is lateral


            //telemetry.log().add("X: " + robot.drive.getTargetVelocity().getX() + " Y: " + robot.drive.getTargetVelocity().getY());
        }

    }
}
