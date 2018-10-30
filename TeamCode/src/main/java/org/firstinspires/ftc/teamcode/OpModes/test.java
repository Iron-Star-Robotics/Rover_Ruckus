package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Motion.RobotTankDrive;

@TeleOp(name="test")
public class test extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotTankDrive tank = new RobotTankDrive(hardwareMap);
        while (opModeIsActive())
            tank.setMotorPowers(.5,.5);
    }
}
