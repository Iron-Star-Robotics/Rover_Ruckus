package org.firstinspires.ftc.teamcode.OpModes.Auto.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.openftc.revextensions2.ExpansionHubMotor;

public class EncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx backLeft = hardwareMap.get(ExpansionHubMotor.class, "bl");

        waitForStart();
        while (opModeIsActive()) {
            backLeft.setPower(.7);
            telemetry.addData("Bl encoder counts: ", backLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}
