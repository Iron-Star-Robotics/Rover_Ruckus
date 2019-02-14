package org.firstinspires.ftc.teamcode.OpModes.TeleOp.tests;
//goo.gl/Qioh8Y
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
//ur mom gae

@Config
@TeleOp(name="SubsystemTest")
public class SubsystemTest extends LinearOpMode {
    Robot robot;
    public static double motorPower = 0.7;
    public static int encoderCts = 750;
    @Override
    public void runOpMode() {
        robot = new Robot(this);

        DcMotorEx liftMotorL = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftMotorL");
        DcMotorEx liftMotorR = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftMotorR");
        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorL.setTargetPosition(0);
        liftMotorR.setTargetPosition(0);
        liftMotorL.setPower(.1);
        liftMotorR.setPower(.1);
        /*
        liftMotorL.setPower(.1);
        liftMotorR.setPower(.1);
        sleep(1000);

        liftMotorL.setPower(0);
        liftMotorR.setPower(0);
        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotorL.setTargetPosition(0);
        liftMotorR.setTargetPosition(0);
        liftMotorL.setPower(.7);
        liftMotorR.setPower(.7);*/

        waitForStart();
        int offsetLeft = liftMotorL.getCurrentPosition();
        int offsetRight = liftMotorR.getCurrentPosition();
        /*
        while (opModeIsActive()) {
            if (gamepad1.y) {
                liftMotorL.setPower(motorPower);
                liftMotorR.setPower(motorPower);
            } else if (gamepad1.a) {
                liftMotorL.setPower(-motorPower);
                liftMotorR.setPower(-motorPower);
            } else {
                liftMotorL.setPower(0);
                liftMotorR.setPower(0);
            }

            telemetry.addData(" Left Encoder count: ", (liftMotorL.getCurrentPosition() - offsetLeft));
            telemetry.addData(" Right Encoder count: ", (liftMotorR.getCurrentPosition() - offsetRight));
            telemetry.update();
        }*/

        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (opModeIsActive()) {
            liftMotorL.setTargetPosition(encoderCts);
            liftMotorR.setTargetPosition(encoderCts);

            liftMotorL.setPower(-.2);
            liftMotorR.setPower(-.2);


            telemetry.addData("Right Encoder count: ", liftMotorR.getCurrentPosition());
            telemetry.update();
            while (liftMotorL.isBusy() || liftMotorR.isBusy()) {

            }
            liftMotorL.setPower(0);
            liftMotorR.setPower(0);
        }

    }



}
