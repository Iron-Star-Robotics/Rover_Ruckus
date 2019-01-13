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
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Utils.Hardware.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Utils.Hardware.CachingServo;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

import java.util.HashMap;
import java.util.Map;

@Config
public class Lift implements Subsystem {
    public static PIDCoefficients LIFT_PID = new PIDCoefficients();

    public static double LIFT_RAISE_HEIGHT = 3; // in
    public static double LIFT_SPOOL_RADIUS = 1 * 1.1; // in
    public static double LIFT_LOWER_HEIGHT = 0; // in
    public static double LIFT_KV = 0;
    public static double ARM_SCORE_POS = 0;
    public static double ARM_RECV_POS = 0;

    public static double LIFT_POWER = 0.8;

    private DigitalChannel liftUp, liftDown;
    // these constants are static and non final for now so they can be modified thru the dashboard as needed
    public static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(NeveRest40Gearmotor.class);

    boolean goingUp = true;

    private PIDFController controller;
    private CachingDcMotorEx liftMotorLeft, liftMotorRight;
    private CachingServo armServo;
    private int encoderOffset;
    public Mode mode = Mode.LIMIT_SWITCH;



    enum Mode {
        RUN_ENCODER,
        STOP,
        PID,
        LIMIT_SWITCH
    }


    public Lift(Robot robot, HardwareMap hardwareMap) {
        controller = new PIDFController(LIFT_PID, LIFT_KV);
        liftMotorLeft = new CachingDcMotorEx(hardwareMap.get(ExpansionHubMotor.class, "liftMotorLeft"));
        liftMotorRight = new CachingDcMotorEx(hardwareMap.get(ExpansionHubMotor.class, "liftMotorRight"));

        if (mode == Mode.LIMIT_SWITCH) {
            liftUp = hardwareMap.get(DigitalChannel.class, "liftUp");
            liftDown = hardwareMap.get(DigitalChannel.class, "liftDown");
        }

        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);


        armServo = new CachingServo(hardwareMap.get(ExpansionHubServo.class, "armServo"));

        robot.addMotor(liftMotorLeft);
        robot.addMotor(liftMotorRight);
        robot.addMotor(armServo);

        encoderOffset = (liftMotorRight.getCurrentPosition() + liftMotorLeft.getCurrentPosition()) / 2;

    }

    private int inchesToTicks(double inches) {
        double ticksPerRev = MOTOR_CONFIG.getTicksPerRev();
        double circumference = 2 * Math.PI * LIFT_SPOOL_RADIUS;
        return (int) Math.round(inches * ticksPerRev / circumference);
    }

    private double ticksToInches(int ticks) {
        double ticksPerRev = MOTOR_CONFIG.getTicksPerRev();
        double revs = ticks / ticksPerRev;
        return 2 * Math.PI * LIFT_SPOOL_RADIUS * revs;
    }

    private double getCurrentHeight() {
        return ticksToInches(getMotorTicks());
    }

    private int getMotorTicks() {
        return (liftMotorLeft.getCurrentPosition() + liftMotorRight.getCurrentPosition()) / 2;
    }

    private void setHeight(double height) {
        if (height > LIFT_RAISE_HEIGHT)
            height = LIFT_RAISE_HEIGHT;
        else if (height < LIFT_LOWER_HEIGHT)
            height = LIFT_LOWER_HEIGHT;
        controller.setTargetPosition(height);
    }

    public void setHeightEncoder(double height) {
        double ticks = inchesToTicks(height);

        liftMotorLeft.setTargetPosition(liftMotorLeft.getCurrentPosition() + inchesToTicks(getMotorTicks()));
        liftMotorRight.setTargetPosition(liftMotorRight.getCurrentPosition() + inchesToTicks(getMotorTicks()));

        liftMotorLeft.setPower(.5);
        liftMotorRight.setPower(.5);

        while (liftMotorLeft.isBusy() || liftMotorRight.isBusy()) {

        }
    }

    @Override
    public Map<String, Object> updateSubsystem(Canvas fieldOverlay) {
        Map<String, Object> t = new HashMap<>();
        if (mode == Mode.PID) {
            t.put("Mode","PID");
            Map<String, Object> telemetryData = new HashMap<>();
            double power = controller.update(getCurrentHeight());
            setLiftPower(power);
            telemetryData.put("Height", getCurrentHeight());
            return telemetryData;
        } else if (mode == Mode.LIMIT_SWITCH){
            t.put("Mode", "Limit Switch");
            if (goingUp) {
                if (liftUp.getState()) {
                    liftMotorLeft.setPower(0);
                    liftMotorRight.setPower(0);
                    mode = Mode.STOP;
                    goingUp = false;
                }
            } else {
                if (liftDown.getState()) {
                    liftMotorLeft.setPower(0);
                    liftMotorRight.setPower(0);
                    mode = Mode.STOP;
                    goingUp = true;
                }
            }
        }

        return t;
    }

    public void setLiftPower(double power) {
        liftMotorRight.setPower(power);
        liftMotorLeft.setPower(power);
    }

    public void raise() {
        if (mode == Mode.PID)
            setHeight(LIFT_RAISE_HEIGHT);
        else {
            setLiftPower(LIFT_POWER);
            while (mode != Mode.STOP);
        }

    }

    public void lower() {
        if (mode == Mode.PID)
            setHeight(LIFT_LOWER_HEIGHT);
        else {
            setLiftPower(-LIFT_POWER);
            while (mode != Mode.STOP); // block until the lift stops moving

        }
    }

    public void score() {
        raise();
        armServo.setPosition(ARM_SCORE_POS);
    }

    public void retract() {
        lower();
        armServo.setPosition(ARM_RECV_POS);
    }

}
