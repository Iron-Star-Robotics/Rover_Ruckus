package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Motion.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Motion.RobotTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.RobotTankDriveBase;
import org.openftc.revextensions2.ExpansionHubMotor;

@Config
@Autonomous(name="testauto")
public class testAuto extends LinearOpMode {
    public static double LEFT_MOTOR_POWER = .1;
    public static double RIGHT_MOTOR_POWER = .1;
    Robot robot;
    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.start();

        waitForStart();
        while (opModeIsActive()) {
            robot.drive.setMotorPowers(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER, RIGHT_MOTOR_POWER, LEFT_MOTOR_POWER);
        }
    }
}
