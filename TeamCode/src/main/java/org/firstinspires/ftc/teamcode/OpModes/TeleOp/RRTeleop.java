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
    public static double motorPower = 0.7;
    public static double NORMAL_DAMPER = 0.8;
    public static double ENDGAME_DAMPER = .3;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.start();


        DcMotorEx liftMotorL = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftMotorL");
        DcMotorEx liftMotorR = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftMotorR");
        liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        while (opModeIsActive()) {
            /*
            robot.drive.setRVelocity(new Pose2d(
                    -(gamepad1.left_stick_x * gamepad1.left_stick_x),
                    -(gamepad1.left_stick_y * gamepad1.left_stick_y),
                    -(gamepad1.right_stick_x * gamepad1.right_stick_x)
            ));*/

            robot.drive.setTargetVelocity(new Pose2d(
                    scalePowers(-gamepad1.left_stick_y),
                    scalePowers(-gamepad1.left_stick_x),
                    scalePowers(-gamepad1.right_stick_x)
            )); // this might seem weird but actually x is axial and y is lateral


            if (gamepad1.y) {
                liftMotorL.setPower(-motorPower);
                liftMotorR.setPower(-motorPower);
            } else if (gamepad1.a) {
                liftMotorL.setPower(motorPower);
                liftMotorR.setPower(motorPower);
            } else {
                liftMotorL.setPower(0);
                liftMotorR.setPower(0);
            }

            telemetry.addData("Left encoder: ", liftMotorL.getCurrentPosition());
            telemetry.addData("Right encoder: ", liftMotorR.getCurrentPosition());
            telemetry.update();



            //telemetry.log().add("X: " + robot.drive.getTargetVelocity().getX() + " Y: " + robot.drive.getTargetVelocity().getY());
        }

    }

    public double scalePowers(double power) {
        double sinPower = Math.sin(power * Math.PI / 2);
        if (gamepad2.left_bumper) return ENDGAME_DAMPER * sinPower;
        return NORMAL_DAMPER * sinPower;
    }
}
