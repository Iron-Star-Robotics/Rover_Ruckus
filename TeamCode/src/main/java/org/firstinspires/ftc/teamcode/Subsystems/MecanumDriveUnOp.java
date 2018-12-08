package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Motion.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Motion.DriveConstants;
import org.firstinspires.ftc.teamcode.Utils.DashboardUtil;
import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MecanumDriveUnOp extends MecanumDriveBase implements Subsystem{
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private double rawHeading, currHeading, lastHeading;
    private BNO055IMU imu;
    private PIDFController headingController;
    private FtcDashboard dashboard;

    public MecanumDriveUnOp(HardwareMap hardwareMap) {
        super();

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        this.dashboard = FtcDashboard.getInstance();
        leftFront = (DcMotorEx) hardwareMap.get(DcMotor.class, "fl");
        leftRear = (DcMotorEx) hardwareMap.get(DcMotor.class, "bl");
        rightRear = (DcMotorEx) (hardwareMap.get(DcMotor.class, "br"));
        rightFront = (DcMotorEx) hardwareMap.get(DcMotor.class, "fr");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, comment out the following line
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: set the tuned coefficients from DriveVelocityPIDTuner if using RUN_USING_ENCODER
        // setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ...);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
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
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(DriveConstants.encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }


    @Override
    public double getExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public FtcDashboard getDashboard() { return dashboard; }

    @Override
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

    // set motor velocity in open loop control
    public void setVelocity(Pose2d target) {
        double v = target.pos().norm();
        v = Range.clip(v, -1, 1);
        double theta = Math.atan2(target.getX(), target.getY());
        double omega = target.getHeading() * 5; // 5 is max heading velo
        Pose2d targetVelocity = new Pose2d(v * Math.cos(theta), v * Math.sin(theta), omega); // determining the needed rotational velo

    }

    public void imuPIDTurn(double radians, double tolerance) {
        headingController.reset();
        headingController.setInputBounds(-Math.PI, Math.PI);
        headingController.setOutputBounds(-1, 1);
        headingController.setTargetPosition(radians);

        while (Math.abs(headingController.getLastError()) >= tolerance) {

        }
    }
}