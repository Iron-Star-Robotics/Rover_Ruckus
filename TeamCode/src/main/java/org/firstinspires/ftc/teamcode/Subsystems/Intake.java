package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Calculator;


public class Intake {
    private final HardwareMap hmap;
    private final DcMotorEx intakeMotor, armMotor;
    private static final double MOTOR_POWER = 0.8;
    private boolean running = false;

    public Intake(final HardwareMap hmap) {
        this.hmap = hmap;
        this.intakeMotor = (DcMotorEx) hmap.dcMotor.get("intakeMotor");
        this.armMotor = (DcMotorEx) hmap.dcMotor.get("armMotor");

    }

    public void pulseIn() {
        intakeMotor.setPower(-MOTOR_POWER);
    }

    public void off() {
        intakeMotor.setPower(0);
    }

    public void pulseOut() {
        intakeMotor.setPower(MOTOR_POWER);
    }



    public void armMove(final double power) {
        runWithEncoder(.5, power);
    }

    public void runWithEncoder(final double revolutions, final double power) {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (power < 0) {
            armMotor.setTargetPosition(-1 * (int) Calculator.encoderRevolutions(revolutions));
        } else {
            armMotor.setTargetPosition((int) Calculator.encoderRevolutions(revolutions));
        }
        armMotor.setPower(power);
    }











}
