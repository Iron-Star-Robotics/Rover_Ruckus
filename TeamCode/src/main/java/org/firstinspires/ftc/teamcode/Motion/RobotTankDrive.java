package org.firstinspires.ftc.teamcode.Motion;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Subsystems.RobotTankDriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Utils.DashboardUtil;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

public class RobotTankDrive extends RobotTankDriveBase implements Subsystem {

    private List<DcMotorEx> motors, leftMotors, rightMotors;
    private BNO055IMU imu;
    private FtcDashboard dashboard;
    private double lastHeading, currHeading, rawHeading;

    public RobotTankDrive(HardwareMap hardwareMap) {
        super();
        dashboard = FtcDashboard.getInstance();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        //DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "br");
        //DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftRear, rightRear);
        leftMotors = Arrays.asList(leftRear);
        rightMotors = Arrays.asList(rightRear);

        for (DcMotorEx motor : motors) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, comment out the following line
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: reverse any motors using DcMotor.setDirection()
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
        for (DcMotorEx leftMotor : leftMotors) {
            leftSum += DriveConstants.encoderTicksToInches(leftMotor.getCurrentPosition());
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightSum += DriveConstants.encoderTicksToInches(rightMotor.getCurrentPosition());
        }
        return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());
    }

    @Override
    public void setMotorPowers(double v, double v1) {
        for (DcMotorEx leftMotor : leftMotors) {
            leftMotor.setPower(v);
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightMotor.setPower(v1);
        }
    }

    @Override
    public double getExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public void gamepadDrive(final double gamepadX, final double gamepadY) {
        for (DcMotorEx leftMotor : leftMotors) {
            leftMotor.setPower(gamepadY + gamepadX);
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightMotor.setPower(gamepadY - gamepadX);
        }
    }

    public BNO055IMU getImu() { return imu; }

    public FtcDashboard getDashboard() {
        return dashboard;
    }

    public TelemetryPacket updateSubsystem() {
        TelemetryPacket packet = new TelemetryPacket();
        if (isFollowingTrajectory()) {
            Pose2d currentPose = getPoseEstimate();
            Pose2d error = getFollowingError();

            Canvas fieldOverlay = packet.fieldOverlay();

            packet.put("xError", error.getX());
            packet.put("yError", error.getY());
            packet.put("headingError", error.getHeading());
            fieldOverlay.setStrokeWidth(4);
            fieldOverlay.setStroke("green");
            DashboardUtil.drawSampledTrajectory(fieldOverlay, getTrajectory());

            fieldOverlay.setFill("blue");
            fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);
            update();

        } else {
            updateHeading();
            packet.put("heading: ", getHeading());
        }


        return packet;
    }

    public void updateHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        rawHeading = angles.firstAngle;
    }

    public void resetAngle() {
        lastHeading = getRawHeading();
        currHeading = 0;
    }

    public double getRawHeading() {
        return rawHeading;
    }

    public double getHeading() {
        double deltaAngle = currHeading - lastHeading;

        if (deltaAngle < -Math.PI)
            deltaAngle += 2 * Math.PI;
        else if (deltaAngle > Math.PI)
            deltaAngle -= 2 * Math.PI;

        currHeading = rawHeading + deltaAngle;
        lastHeading = currHeading;
        return currHeading;
    }






}
