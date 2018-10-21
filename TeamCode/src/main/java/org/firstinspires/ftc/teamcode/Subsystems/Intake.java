package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Motion.PID;

public class Intake {
    private final HardwareMap hmap;
    private PID pid;
    private final DcMotorEx intakeMotor;
    private static final double MOTOR_POWER = 0.5;
    private boolean running = false;
    public Intake(final HardwareMap hmap) {
        this.hmap = hmap;
        this.intakeMotor = (DcMotorEx) hmap.dcMotor.get("intakeMotor");
    }

    public void pulse() {
        if (!running) {
            intakeMotor.setPower(MOTOR_POWER);
        } else {
            intakeMotor.setPower(0);
        }
    }




}
