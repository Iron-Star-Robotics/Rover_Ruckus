package org.firstinspires.ftc.teamcode.Utils.Hardware;


import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorUtils {
    public static void runToPosMotorInit(CachingDcMotorEx delegate) {
        delegate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        delegate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        delegate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // assumes in run to pos mode
    public static void runToPos(CachingDcMotorEx delegate, int ticks, double power) {
        delegate.setTargetPosition(ticks);
        delegate.setPower(power);
    }
}
