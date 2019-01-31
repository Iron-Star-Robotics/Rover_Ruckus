package org.firstinspires.ftc.teamcode.Utils.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.revextensions2.ExpansionHubMotor;

public class HoldingMotor extends CachingDcMotorEx {
    private int targetPos = 0;
    private double maxPower;
    double dir;

    public HoldingMotor(ExpansionHubMotor delegate, double maxPower) {
        super(delegate);
        this.maxPower = maxPower;
    }

    public void setTargetPos(int targetPos) {
        this.targetPos = targetPos;
        delegate.setMode(RunMode.STOP_AND_RESET_ENCODER);
        delegate.setMode(RunMode.RUN_TO_POSITION);

        delegate.setTargetPosition(targetPos);
    }

    public void hold() {
        delegate.setPower(maxPower);
    }

    public void release() {
        delegate.setPower(0);
        delegate.setMode(RunMode.RUN_USING_ENCODER);
    }


}
