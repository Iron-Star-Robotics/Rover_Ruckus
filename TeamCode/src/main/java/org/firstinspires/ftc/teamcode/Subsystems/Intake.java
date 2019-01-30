package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
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

@Config
public class Intake implements Subsystem {
    CachingDcMotorEx flipper, collector;

    public static int flipTicks = 200;
    public Intake(HardwareMap map) {
        flipper = new CachingDcMotorEx(map.get(ExpansionHubMotor.class, "flipper"));
        collector = new CachingDcMotorEx(map.get(ExpansionHubMotor.class, "collector"));
    }

    @Override
    public Map<String, Object> updateSubsystem(Canvas fieldOverlay) {
        Map<String, Object> telemetry = new HashMap<>();
        telemetry.put("Intake state", "lol");
        return telemetry;
    }

    public void flipOut() {
        flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipper.setTargetPosition(flipTicks);

        flipper.setPower(1);
        while (flipper.isBusy());
        flipper.setPower(0);
    }

    public void flipIn() {

    }




}
