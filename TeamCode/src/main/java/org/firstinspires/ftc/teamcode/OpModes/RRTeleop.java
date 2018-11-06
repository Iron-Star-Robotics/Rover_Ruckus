package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.RobotTankDriveOptimized;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp(name="RRTeleop")

public class RRTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotTankDriveOptimized drive = new RobotTankDriveOptimized(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            drive.gamepadDrive(gamepad1.right_stick_x, -gamepad1.left_stick_y);
            if (gamepad1.right_trigger >= .4) {
                intake.pulseIn();
            }

            else if (gamepad1.left_trigger >= .4) {
                intake.pulseOut();
            }

            else intake.off();

            if (gamepad1.right_bumper) {
                intake.armMove(.3);
            } else if (gamepad1.left_bumper) { intake.armMove(-.3);};

        }
    }
}
