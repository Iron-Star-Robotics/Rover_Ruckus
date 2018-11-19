package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Motion.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Motion.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.Utils.LynxOptimizedI2cFactory;
import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Optimized mecanum drive that can cut loop times in half
 *
 */

public class MecanumDrive extends MecanumDriveBase implements Subsystem {
    private ExpansionHubEx hub;
    private CachingDcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<CachingDcMotorEx> motors;
    private BNO055IMU imu;
    private double rawHeading, currHeading, lastHeading;
    private FtcDashboard dashboard;

    public MecanumDrive(HardwareMap hardwareMap) {
        super();

        RevExtensions2.init();
        this.dashboard = FtcDashboard.getInstance();

        hub = hardwareMap.get(ExpansionHubEx.class, "hub");

        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        try {
            // axis remap
            byte AXIS_MAP_CONFIG_BYTE = 0b00011000; //swaps y-z, 0b00100001 is y-x, 0x6 is x-z
            byte AXIS_MAP_SIGN_BYTE = 0b000; //x, y, z

            //Need to be in CONFIG mode to write to registers
            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

            Thread.sleep(100); //Changing modes requires a delay before doing anything else

            //Write to the AXIS_MAP_CONFIG register
            imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);

            //Write to the AXIS_MAP_SIGN register
            imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);

            //Need to change back into the IMU mode to use the gyro
            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);

            Thread.sleep(100); //Changing modes again requires a delay
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        leftFront = new CachingDcMotorEx(hardwareMap.get(ExpansionHubMotor.class, "leftFront"));
        leftRear = new CachingDcMotorEx(hardwareMap.get(ExpansionHubMotor.class, "leftRear"));
        rightRear = new CachingDcMotorEx(hardwareMap.get(ExpansionHubMotor.class, "rightRear"));
        rightFront = new CachingDcMotorEx(hardwareMap.get(ExpansionHubMotor.class, "rightFront"));

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (CachingDcMotorEx motor : motors) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, comment out the following line
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

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
        for (CachingDcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();
        List<Double> wheelPositions = new ArrayList<>();
        for (CachingDcMotorEx motor : motors) {
            wheelPositions.add(DriveConstants.encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
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
}
