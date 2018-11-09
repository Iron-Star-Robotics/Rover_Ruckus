package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Motion.RobotTankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.RobotTankDriveBase;

@Autonomous(name="testauto")
public class testAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotTankDriveBase drive = new RobotTankDrive(hardwareMap);
        waitForStart();
        while (opModeIsActive())
            drive.setMotorPowers(1.0, 1.0);
    }
}
