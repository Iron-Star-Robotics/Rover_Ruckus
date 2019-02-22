package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utils.Misc.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.Utils.Misc.Smoother;

@TeleOp(name="RRTeleop")
@Config

public class RRTeleop extends LinearOpMode {
    Robot robot;
    public static double motorPower = 0.7;
    public static double NORMAL_DAMPER = 0.8;
    public static double ENDGAME_DAMPER = .3;
    public static final double AXIAL_SMOOTHING_COEFF = .9;
    public static final double LATERAL_SMOOTHING_COEFF = .7;
    public static final double TURN_SMOOTH_COEFF = .5;

    DcMotorEx liftMotor, intake, flipper, lift;
    CRServo servo;
    Servo scoreOWO;
    Smoother smoother;

    boolean fieldCentric;

    @Override
    public void runOpMode() {
       smoother = new Smoother();
       smoother.setMode(Smoother.MODE.LINEAR);

        fieldCentric = false;

        robot = new Robot(this);
        robot.start();
        liftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftMotor");
        flipper = (DcMotorEx) hardwareMap.get(DcMotor.class, "flipper");
        intake = (DcMotorEx) hardwareMap.get(DcMotor.class, "intake");
        scoreOWO = hardwareMap.get(Servo.class, "scorer");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = (DcMotorEx) hardwareMap.get(DcMotor.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo = hardwareMap.get(CRServo.class, "collector");

        waitForStart();
        while (opModeIsActive()) {
            long startTime = System.currentTimeMillis();
            Pose2d controllerPose = new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x * .75
            );

            robot.drive.setTargetVelocity(smoother.transform(controllerPose));

            if (gamepad1.y)
                liftMotor.setPower(1.0);
            else if (gamepad1.a)
                liftMotor.setPower(-1.0);
            else
                liftMotor.setPower(0);

            telemetry.addData("Lift count: ", liftMotor.getCurrentPosition());
            telemetry.update();
            //telemetry.log().add("X: " + robot.drive.getTargetVelocity().getX() + " Y: " + robot.drive.getTargetVelocity().getY());

            if (gamepad1.x)
                intake.setPower(.2);
            else if (gamepad1.b)
                intake.setPower(-.2);
            else
                intake.setPower(0);
            if (gamepad1.right_trigger > .2) {
                flipper.setPower(.7);
            } else if (gamepad1.left_trigger > .2)
                flipper.setPower(-.7);
            else
                flipper.setPower(0);


            if (gamepad1.right_bumper)
                lift.setPower(.2);
            else if (gamepad1.left_bumper)
                lift.setPower(-.2);
            else
                lift.setPower(0);
            if (gamepad1.dpad_up)
                servo.setPower(1);
            else if (gamepad1.dpad_down)
                servo.setPower(-.1);
            else
                servo.setPower(0);

            if (gamepad1.dpad_left) {
                scoreOWO.setPosition(1);
            } else if (gamepad1.dpad_right)
                scoreOWO.setPosition(.2);


            telemetry.addData("intake counts: ", intake.getCurrentPosition());
            telemetry.addData("flipper counts: ", flipper.getCurrentPosition());
            telemetry.addData("score lift counts: ", lift.getCurrentPosition());


            long endTime = System.currentTimeMillis();
            double loopTimeHz = 1000.0 / (endTime - startTime);
            telemetry.addData("Loop Time (Hz):  ", loopTimeHz);
            telemetry.update();
        }


    }

    public void setFieldCentric() {

    }



}
