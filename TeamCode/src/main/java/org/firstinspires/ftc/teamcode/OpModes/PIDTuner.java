package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Motion.RobotTankDrive;

public class PIDTuner extends LinearOpMode {

    public static PIDFCoefficients MOTOR_PIDF = new PIDFCoefficients();
    public static double DISTANCE = 72;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        RobotTankDrive drive = new RobotTankDrive(hardwareMap);

        PIDFCoefficients currentCoeffs = drive.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        MOTOR_PIDF = pidfCopy(currentCoeffs);
        dashboard.updateConfig();

        RobotLog.i("Initial motor PIDF coefficients: " + MOTOR_PIDF);

        NanoClock clock = NanoClock.system();

        telemetry.log().add("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();



    }

    private static PIDFCoefficients pidfCopy(PIDFCoefficients coeff) {
        return new PIDFCoefficients(coeff.p, coeff.i, coeff.d, coeff.f, coeff.algorithm);
    }

}
