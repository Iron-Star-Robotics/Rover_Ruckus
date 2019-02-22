package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Hardware.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Utils.Hardware.CachingServo;
import org.firstinspires.ftc.teamcode.Utils.Hardware.MotorUtils;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

import java.util.HashMap;
import java.util.Map;

@Config
public class Lift implements Subsystem {
    private CachingDcMotorEx liftMotor;
    private CachingServo scoringServo;

   // Scoring lift
    public static int scoreTicks = 1000;
    public static double scoringPos = .7;

    private boolean lifting = false;

    public Lift(Robot robot, HardwareMap hardwareMap) {
        liftMotor = new CachingDcMotorEx(hardwareMap.get(ExpansionHubMotor.class, "lift"));
        MotorUtils.runToPosMotorInit(liftMotor);
        robot.addMotor(liftMotor);
        scoringServo = new CachingServo(hardwareMap.get(ExpansionHubServo.class, "scorer"));
        robot.addMotor(scoringServo);
    }

    @Override
    public Map<String, Object> updateSubsystem(Canvas fieldOverlay) {
        if (lifting && !liftMotor.isBusy()) {
            scoringServo.setPosition(scoringPos);
        }
        return new HashMap<>();
    }

    public void score() {
        liftMotor.setTargetPosition(scoreTicks);
        liftMotor.setPower(.5);
        lifting = true;
    }

    public void retract() {
        liftMotor.setTargetPosition(10);
        liftMotor.setPower(.5);
        lifting = false;
    }
}
