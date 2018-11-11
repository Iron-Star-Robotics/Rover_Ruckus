package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@TeleOp(name="Servotest")
public class ServoTurnTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);
        while (opModeIsActive()) {
            if (gamepad1.a)
                robot.drive.getServo().setPosition(-1.0);
            if (gamepad1.b)
                robot.drive.getServo().setPosition(1.0);
            sleep(50);
        }
    }
}
