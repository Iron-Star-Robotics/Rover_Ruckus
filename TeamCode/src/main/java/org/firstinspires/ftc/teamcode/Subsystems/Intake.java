package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Utils.Hardware.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Utils.Hardware.CachingMotor;
import org.firstinspires.ftc.teamcode.Utils.Hardware.CachingServo;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

import java.util.HashMap;
import java.util.Map;

public class Intake implements Subsystem {
    public static PIDCoefficients INTAKE_PID = new PIDCoefficients();

    public static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(NeveRest40Gearmotor.class);

    public static double INTAKE_SPOOL_RADIUS = 1; // in
    public static double INTAKE_RETRACT_LEN = 0; // in
    public static double INTAKE_EXTEND_LEN = 8; // in
    public static double INTAKE_KV = 0;

    private PIDFController controller;
    private CachingDcMotorEx intakeMotor;
    private CachingServo collectionServo, flipServoLeft, flipServoRight;

    private double desiredExtension;

    public Intake(Robot robot, HardwareMap hardwareMap) {
        intakeMotor = new CachingDcMotorEx(hardwareMap.get(ExpansionHubMotor.class, "intakeMotor"));
        collectionServo = new CachingServo(hardwareMap.get(ExpansionHubServo.class, "collectionServo"));
        flipServoLeft = new CachingServo(hardwareMap.get(ExpansionHubServo.class, "flipServoLeft"));
        flipServoRight = new CachingServo(hardwareMap.get(ExpansionHubServo.class, "flipServoRight"));

        robot.addMotor(intakeMotor);
        robot.addMotor(collectionServo);
        robot.addMotor(flipServoLeft);
        robot.addMotor(flipServoRight);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new PIDFController(INTAKE_PID, INTAKE_KV);
    }

    private double ticksToInches(int ticks) {
        double ticksPerRev = MOTOR_CONFIG.getTicksPerRev();
        double revs = ticks / ticksPerRev;
        return 2 * Math.PI * INTAKE_SPOOL_RADIUS * revs;
    }

    public double getCurrentHeight() {
        return ticksToInches(intakeMotor.getCurrentPosition());
    }

    public void setExtension(double extension) {
        desiredExtension = extension;
        controller.setTargetPosition(desiredExtension);
    }

    @Override
    public Map<String, Object> updateSubsystem(Canvas fieldOverlay) {
        Map<String, Object> telemetryData = new HashMap<>();
        double power = controller.update(getCurrentHeight());
        intakeMotor.setPower(power);
        telemetryData.put("Height", getCurrentHeight());
        return telemetryData;
    }


}
