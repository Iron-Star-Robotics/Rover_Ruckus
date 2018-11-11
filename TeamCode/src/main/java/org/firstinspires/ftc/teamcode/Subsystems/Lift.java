package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Calculator;

import java.util.Map;

public class Lift implements Subsystem {
    private DcMotorEx liftMotor;

    public Lift(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public TelemetryPacket updateSubsystem() {
        return new TelemetryPacket();
    }

    public void encoderMove(double distance, double power) {
        int encoderDist = Calculator.Inches2Encoder(distance);
        liftMotor.setPower(power);
        liftMotor.setTargetPosition(liftMotor.getCurrentPosition() + encoderDist);
    }


}
