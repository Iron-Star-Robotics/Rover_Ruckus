package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Motion.RobotTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.RobotTankDriveBase;

@Config
@Autonomous(name="testauto")
public class testAuto extends LinearOpMode {
    public static double LEFT_MOTOR_POWER = .1;
    public static double RIGHT_MOTOR_POWER = .1;
    @Override
    public void runOpMode() {
        RobotTankDriveBase drive = new RobotTankDrive(hardwareMap);
        waitForStart();
        while (opModeIsActive())
            drive.setMotorPowers(LEFT_MOTOR_POWER, RIGHT_MOTOR_POWER);
    }
}
