package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Utils.Hardware.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Utils.Hardware.CachingMotor;
import org.firstinspires.ftc.teamcode.Utils.Hardware.CachingServo;
import org.firstinspires.ftc.teamcode.Utils.Hardware.MotorUtils;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

import java.util.HashMap;
import java.util.Map;

@Config
public class Intake implements Subsystem {
    CachingDcMotorEx flipper, intake;
    CRServo collector;

    public static int extendTicks = 1000;
    public static int flipTicks = 10;
    private boolean needsFlip, flippedOut = false;

    public Intake(Robot robot, HardwareMap map) {
        flipper = new CachingDcMotorEx(map.get(ExpansionHubMotor.class, "flipper"));
        MotorUtils.runToPosMotorInit(flipper);
        intake = new CachingDcMotorEx(map.get(ExpansionHubMotor.class, "intake"));
        MotorUtils.runToPosMotorInit(intake);
        collector = map.get(CRServo.class, "collector");
    }

    @Override
    public Map<String, Object> updateSubsystem(Canvas fieldOverlay) {
        Map<String, Object> telemetry = new HashMap<>();
        telemetry.put("Intake state", "lol");
        return telemetry;

    }

    public void collect() {
        collector.setPower(1);
    }

    public void spit() {
        collector.setPower(-1);
    }

    public void extend() {
        intake.setTargetPosition(extendTicks);
        intake.setPower(.4);
        flipper.setTargetPosition(flipTicks);
        flipper.setPower(.7);
    }

    public void retract() {
        intake.setTargetPosition(10);
        intake.setPower(.4);
        needsFlip = true;
        flipper.setTargetPosition(0);
        flipper.setPower(.7);
    }

}
