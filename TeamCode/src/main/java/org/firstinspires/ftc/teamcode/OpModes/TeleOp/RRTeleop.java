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

    DcMotorEx liftMotor;


    Smoother smoother;

    boolean fieldCentric;

    @Override
    public void runOpMode() {
       smoother = new Smoother();
       smoother.setMode(Smoother.MODE.EXPONENTIAL);

        fieldCentric = false;

        robot = new Robot(this);
        robot.start();
        liftMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "liftMotor");
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
            long endTime = System.currentTimeMillis();
            telemetry.addData("Loop Time (Hz):  ", 1 / ((endTime - startTime) / 1000));
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
