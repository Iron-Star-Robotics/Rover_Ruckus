package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.hardware.motors.RevRobotics40HdHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Utils.Hardware.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Utils.Hardware.CachingServo;

import org.firstinspires.ftc.teamcode.Utils.Hardware.HoldingMotor;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Callable;

@Config
public class Lift implements Subsystem {
    CachingDcMotorEx liftMotor;
    private static final int DELATCH_COUNT = 3400;
    private static final int FAST_RESET_COUNT = 2800;
    enum State {
        STOP,
        FOLLOWING,
        HOLD
    }

    private State state = State.STOP;

    public Lift(Robot robot, HardwareMap hardwareMap) {
        liftMotor = new CachingDcMotorEx(hardwareMap.get(ExpansionHubMotor.class, "liftMotor"));
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.addMotor(liftMotor);
    }

    public void delatch() {
        liftMotor.setTargetPosition(DELATCH_COUNT);
        liftMotor.setPower(1.0);
        state = State.FOLLOWING;
    }

    public void lower() {
        liftMotor.setTargetPosition(10);
        liftMotor.setPower(-1.0);
        state = State.FOLLOWING;
    }

    public void holdBlocking(int counts, Callable<Boolean> func) throws Exception {
        liftMotor.setTargetPosition(counts);
        liftMotor.setPower(.8);
        while (!func.call()) {
            while (liftMotor.isBusy());
            liftMotor.setTargetPosition(counts);
            liftMotor.setPower(.8);
        }
    }

    public void holdAsync() {

    }


    @Override
    public Map<String, Object> updateSubsystem(Canvas fieldOverlay) {
        if (state == State.FOLLOWING) {
            if (!liftMotor.isBusy()) {
                liftMotor.setPower(0);
                state = State.STOP;
            }
        }
        return new HashMap<>();
    }
}
