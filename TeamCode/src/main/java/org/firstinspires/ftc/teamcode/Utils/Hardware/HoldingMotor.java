package org.firstinspires.ftc.teamcode.Utils.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.revextensions2.ExpansionHubMotor;

public class HoldingMotor extends CachingDcMotorEx {
    private int targetPos = 0;
    private double maxPower = 0.7;
    double dir;

    public HoldingMotor(ExpansionHubMotor delegate, double maxPower) {
        super(delegate);
        this.maxPower = maxPower;

        delegate.setMode(RunMode.STOP_AND_RESET_ENCODER);
        delegate.setMode(RunMode.RUN_TO_POSITION);

    }

    public void setTargetPos(int targetPos) {
        this.targetPos = targetPos;
    }

    public void hold() {
        delegate.setTargetPosition(targetPos);
        delegate.setPower(maxPower);
    }

    public void release() {
        delegate.setPower(0);
    }


}
