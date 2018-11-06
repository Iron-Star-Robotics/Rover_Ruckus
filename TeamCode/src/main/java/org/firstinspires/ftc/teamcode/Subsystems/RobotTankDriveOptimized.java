package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Motion.DriveConstants;
import org.firstinspires.ftc.teamcode.Utils.LynxOptimizedI2cFactory;
import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.util.Arrays;
import java.util.List;

// optimized drive train class
// reduces loop times by half by cutting down unnecessary overhead in rev bulk reads

public class RobotTankDriveOptimized extends RobotTankDriveBase {

    private ExpansionHubEx hub;
    private List<ExpansionHubMotor> motors, leftMotors, rightMotors;
    private BNO055IMU imu;

    public RobotTankDriveOptimized(HardwareMap hardwareMap) {
        super();
        RevExtensions2.init();

        hub = hardwareMap.get(ExpansionHubEx.class, "hub");

        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        //DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        ExpansionHubMotor leftRear = hardwareMap.get(ExpansionHubMotor.class, "leftRear");
        ExpansionHubMotor rightRear = hardwareMap.get(ExpansionHubMotor.class, "rightRear");
        //DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftRear, rightRear);
        leftMotors = Arrays.asList(leftRear);
        rightMotors = Arrays.asList(rightRear);

        for (ExpansionHubMotor motor : motors) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if we keep it, DO RUN_WITH_ENCODER
            // otherwise, comment out the following line
            // atm we plan to use built in pid
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        // .08841
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftMotors.get(0).getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        double leftSum = 0, rightSum = 0;
        RevBulkData bulkData = hub.getBulkInputData();
        for (DcMotorEx leftMotor : leftMotors) {
            leftSum += DriveConstants.encoderTicksToInches(bulkData.getMotorCurrentPosition(leftMotor));
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightSum += DriveConstants.encoderTicksToInches(bulkData.getMotorCurrentPosition(rightMotor));
        }
        return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());
    }

    @Override
    public void setMotorPowers(double v, double v1) {
        for (ExpansionHubMotor leftMotor : leftMotors) {
            leftMotor.setPower(v);
        }
        for (ExpansionHubMotor rightMotor : rightMotors) {
            rightMotor.setPower(v1);
        }
    }

    @Override
    public double getExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }


    public void gamepadDrive(double gamepadX, double gamepadY) {
        for (ExpansionHubMotor leftMotor : leftMotors) {
            leftMotor.setPower(gamepadY + gamepadX);
        }
        for (ExpansionHubMotor rightMotor : rightMotors) {
            rightMotor.setPower(gamepadY - gamepadX);
        }
    }




}
