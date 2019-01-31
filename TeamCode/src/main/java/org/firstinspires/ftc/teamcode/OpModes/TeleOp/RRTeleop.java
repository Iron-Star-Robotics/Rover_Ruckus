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

    ExponentialSmoother axialSmoother;
    ExponentialSmoother lateralSmoother;
    ExponentialSmoother turningSmoother;


    @Override
    public void runOpMode() {
        axialSmoother = new ExponentialSmoother(AXIAL_SMOOTHING_COEFF);
        lateralSmoother = new ExponentialSmoother(LATERAL_SMOOTHING_COEFF);
        turningSmoother = new ExponentialSmoother(TURN_SMOOTH_COEFF);
        axialSmoother.reset();
        lateralSmoother.reset();
        turningSmoother.reset();

        robot = new Robot(this);
        robot.start();

        waitForStart();
        while (opModeIsActive()) {
            robot.drive.setTargetVelocity(new Pose2d(
                    axialSmoother.update(-gamepad1.left_stick_y),
                    lateralSmoother.update(-gamepad1.left_stick_x),
                    turningSmoother.update(-gamepad1.right_stick_x)
            ));

            //telemetry.log().add("X: " + robot.drive.getTargetVelocity().getX() + " Y: " + robot.drive.getTargetVelocity().getY());
        }
    }

    public double scalePowers(double power) {
        double sinPower = Math.sin(power * Math.PI / 2);
        if (gamepad2.left_bumper) return ENDGAME_DAMPER * sinPower;
        return NORMAL_DAMPER * sinPower;
    }

}
