package org.firstinspires.ftc.teamcode.OpModes.TeleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import static org.firstinspires.ftc.teamcode.Subsystems.Lift.LIFT_SPOOL_RADIUS;
import static org.firstinspires.ftc.teamcode.Subsystems.Lift.MOTOR_CONFIG;

@TeleOp(name="SubsystemTest")
public class SubsystemTest extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        waitForStart();

        robot.lift.raise();
        sleep(3000);
        robot.lift.lower();


    }



}
