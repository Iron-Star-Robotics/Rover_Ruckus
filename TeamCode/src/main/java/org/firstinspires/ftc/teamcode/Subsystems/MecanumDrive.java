package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.Utils.Hardware.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Utils.Hardware.IMU;
import org.firstinspires.ftc.teamcode.Utils.Misc.CSVWriter;
import org.firstinspires.ftc.teamcode.Utils.Misc.DashboardUtil;
import org.firstinspires.ftc.teamcode.Utils.Hardware.LynxOptimizedI2cFactory;
import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.firstinspires.ftc.teamcode.Motion.DriveConstants;

/**
 * Optimized mecanum drive that can cut loop times in half
 *
 */

@Config
public class MecanumDrive extends MecanumDriveBase implements Subsystem {
    private ExpansionHubEx hub;
    private CachingDcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<CachingDcMotorEx> motors;
    private double bias = 0;
    private IMU imu;
    private Pose2d targetVelocity = new Pose2d(0,0,0);
    public static int ff = 0;

    private List<String> motorNames = new ArrayList<>();

    private static final Vector2d[] wheelPositions = {
            new Vector2d(8,8),
            new Vector2d(-8, 8),
            new Vector2d(-8,-8),
            new Vector2d(8, -8)
    };


    private static final Vector2d[] rotorDirections = {
            new Vector2d(1,-1),
            new Vector2d(1, 1),
            new Vector2d(1, -1),
            new Vector2d(1, 1)
    };

    public static double axialMaxV = 50;
    public static double headingMaxV = 5;

    private PIDFController headingController;


    private static final double TAU = Math.PI * 2;

    public MecanumDrive(Robot robot, HardwareMap hardwareMap) {
        super();

        hub = hardwareMap.get(ExpansionHubEx.class, "hub");
        /*
        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        /*
        try {
            // axis remap
            byte AXIS_MAP_CONFIG_BYTE = 0b00100001; //swaps y-z, 0b00100001 is y-x, 0x6 is x-z
            byte AXIS_MAP_SIGN_BYTE = 0b000; //x, y, zv

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
        }*/

        imu = new IMU(IMU.IMU_POS.VERTICAL, hub);
        imu.init();

        leftFront = new CachingDcMotorEx(hardwareMap.get(ExpansionHubMotor.class, "fl"));
        leftRear = new CachingDcMotorEx(hardwareMap.get(ExpansionHubMotor.class, "bl"));
        rightRear = new CachingDcMotorEx(hardwareMap.get(ExpansionHubMotor.class, "br"));
        rightFront = new CachingDcMotorEx(hardwareMap.get(ExpansionHubMotor.class, "fr"));

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        motorNames.add("fl");
        motorNames.add("bl");
        motorNames.add("br");
        motorNames.add("fr");

        for (CachingDcMotorEx motor : motors) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, comment out the following line
            robot.addMotor(motor);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
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
        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }
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
        return imu.getRawAngle();
    }

    public void setBias(double radians) {
        imu.setBias(radians);
    }

    @Override
    public Map<String, Object> updateSubsystem(Canvas fieldOverlay) {
        Map<String, Object> telemetryData = new HashMap<>();
        imu.update();
        Pose2d currentPose = getPoseEstimate();
        if (isFollowingTrajectory()) {
            Pose2d error = getFollowingError();

            telemetryData.put("xError", error.getX());
            telemetryData.put("yError", error.getY());
            telemetryData.put("headingError",error.getHeading());
            fieldOverlay.setStrokeWidth(4);
            fieldOverlay.setStroke("green");
            DashboardUtil.drawSampledTrajectory(fieldOverlay, getTrajectory());

            fieldOverlay.setFill("blue");
            fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);
            update();
        } else if (ff == 0) {
            updatePoseEstimate(); // we don't need to update the follower here
            setVelocity(getTargetVelocity());
        }

        telemetryData.put("xPos", currentPose.getX());
        telemetryData.put("yPos", currentPose.getY());
        telemetryData.put("heading",currentPose.getHeading());

        for (int i = 0; i < motors.size(); i++) {
            CachingDcMotorEx motor = motors.get(i);
            telemetryData.put(motorNames.get(i) + "encoder counts", motor.getCurrentPosition());
        }

        //internalSetVelocity(targetVelocity);


        return telemetryData;
    }

    public Pose2d getTargetVelocity() {
        return targetVelocity;
    }

    public void setTargetVelocity(Pose2d newVelo) {
        targetVelocity = newVelo;
    }

    public void followFullTrajectory(Trajectory trajectory) {
        setTrajectory(trajectory);
        followTrajectory(trajectory);

        while(isFollowingTrajectory());

    }

    public void followCompositeTrajectory(List<Trajectory> trajectories) {
        for (Trajectory trajectory : trajectories)
            followFullTrajectory(trajectory);
    }


    public double getHeading() {
        return imu.getAngle();
    }
    /*

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
    */
    // set motor velocity in open loop control


    public void setRVelocity(Pose2d target) {
        double controllerAngle = Math.atan2(target.getX(), target.getY());
        double robotAngle = imu.getRawAngle();

        double finalAngle = norm(robotAngle + controllerAngle);
        targetVelocity = new Pose2d(target.getX(), target.getY(), finalAngle);
    }

    private double norm(double angle) {
        double newAngle = angle % TAU;

        newAngle = (newAngle + TAU) % TAU;

        if (newAngle > Math.PI)
            newAngle -= TAU;

        return newAngle;
    }
/*
    public void imuPIDTurn(double radians, double tolerance) {
        headingController.reset();
        headingController.setInputBounds(-Math.PI, Math.PI);
        headingController.setOutputBounds(-1, 1);
        headingController.setTargetPosition(radians);

        while (Math.abs(headingController.getLastError()) >= tolerance) {
            
        }
    }

    private void internalSetVelocity (Pose2d v) {
        for (int i = 0; i < motors.size(); i++) {
            Vector2d rotorVelocity = new Vector2d(
                    v.getX() - v.getHeading() * wheelPositions[i].getY(),
                    v.getY() + v.getHeading() * wheelPositions[i].getX()
            );
            double surfaceVelocity = rotorVelocity.dot(rotorDirections[i]);
            double wheelVelocity = surfaceVelocity / radius;
            //telemetry.log().add("wheelVelo: " + wheelVelocity);
            motors.get(i).setVelocity(wheelVelocity, AngleUnit.RADIANS);
        }
    }*/

    // gonna use roadrunner for pose updates to clean this up a bit
    public double getCurrHeading() {
        return getPoseEstimate().getHeading();
    }


    public void turnTo(double degrees) {
        double radians = Math.toRadians(degrees);
        Trajectory trajectory = trajectoryBuilder().turnTo(radians).build();
        setTrajectory(trajectory);
        followTrajectory(trajectory);
        while (isFollowingTrajectory());
    }

    public void forward(double distance) {
        Trajectory trajectory = trajectoryBuilder().forward(distance).build();
        setTrajectory(trajectory);
        followTrajectory(trajectory);
    }

    public void strafeLeft(double distance) {
        Trajectory trajectory = trajectoryBuilder().strafeLeft(distance).build();
        setTrajectory(trajectory);
        followTrajectory(trajectory);
    }

    public void strafeRight(double distance) {
        Trajectory trajectory = trajectoryBuilder().strafeRight(distance).build();
        setTrajectory(trajectory);
        followTrajectory(trajectory);
    }

    public void lineTo(Vector2d dest) { // line to a place with heading correction
        Trajectory trajectory = trajectoryBuilder().lineTo(dest).build();
        setTrajectory(trajectory);
        followTrajectory(trajectory);
    }
}
