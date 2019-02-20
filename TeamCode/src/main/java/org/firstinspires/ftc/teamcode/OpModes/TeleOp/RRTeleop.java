package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    DcMotorEx liftMotor, intakeMotor, collector;
    Servo servo;

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
        collector = (DcMotorEx) hardwareMap.get(DcMotor.class, "collector");
        intakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo = hardwareMap.get(Servo.class, "flipIn");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            long startTime = System.currentTimeMillis();
            Pose2d controllerPose = new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
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
                intakeMotor.setPower(.6);
            else if (gamepad1.b)
                intakeMotor.setPower(-.6);
            else
                intakeMotor.setPower(0);

            if (gamepad1.left_trigger > .5)
                collector.setPower(1);
            else if (gamepad1.right_trigger > .5)
                collector.setPower(-1);
            else
                collector.setPower(0);

            if (gamepad1.right_bumper)
                servo.setPosition(.5);
            else if (gamepad1.left_bumper)
                servo.setPosition(0);

            long endTime = System.currentTimeMillis();
            double loopTimeHz =  1 / ((endTime - startTime) / 1000.0);
            telemetry.addData("Loop Time (Hz):  ", loopTimeHz);
        }


    }

    public double scalePowers(double power) {
        double sinPower = Math.sin(power * Math.PI / 2);
        if (gamepad2.left_bumper) return ENDGAME_DAMPER * sinPower;
        return NORMAL_DAMPER * sinPower;
    }

    public void setFieldCentric() {

    }



}
