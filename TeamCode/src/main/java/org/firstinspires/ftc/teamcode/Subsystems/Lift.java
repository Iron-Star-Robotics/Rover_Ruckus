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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils.Hardware.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Utils.Hardware.CachingServo;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

import java.util.HashMap;
import java.util.Map;

@Config
public class Lift implements Subsystem {
    public static PIDCoefficients LIFT_PID = new PIDCoefficients();
    public static double LIFT_RAISE_HEIGHT = 10; // in
    public static double LIFT_SPOOL_RADIUS = 1; // in
    public static double LIFT_LOWER_HEIGHT = 0; // in
    public static double LIFT_KV = 0;
    public static double ARM_SCORE_POS = 0;
    public static double ARM_RECV_POS = 0;
    public static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(NeveRest40Gearmotor.class);


    private PIDFController controller;
    private CachingDcMotorEx liftMotorLeft, liftMotorRight;
    private CachingServo armServo;
    private int encoderOffset;


    public Lift(Robot robot, HardwareMap hardwareMap) {
        controller = new PIDFController(LIFT_PID, LIFT_KV);
        liftMotorLeft = new CachingDcMotorEx(hardwareMap.get(ExpansionHubMotor.class, "liftMotorLeft"));
        liftMotorRight = new CachingDcMotorEx(hardwareMap.get(ExpansionHubMotor.class, "liftMotorRight"));

        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    @Override
    public Map<String, Object> updateSubsystem(Canvas fieldOverlay) {
        Map<String, Object> telemetryData = new HashMap<>();
        double power = controller.update(getCurrentHeight());
        setLiftPower(power);
        telemetryData.put("Height", getCurrentHeight());
        return telemetryData;
    }

    public void setLiftPower(double power) {
        liftMotorRight.setPower(power);
        liftMotorLeft.setPower(power);
    }

    public void raise() {
        setHeight(LIFT_RAISE_HEIGHT);
    }

    public void lower() {
        setHeight(LIFT_LOWER_HEIGHT);
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
