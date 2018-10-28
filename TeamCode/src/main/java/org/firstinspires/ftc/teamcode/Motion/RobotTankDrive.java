package org.firstinspires.ftc.teamcode.Motion;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.teamcode.Utils.Calculator;

import java.util.ArrayList;
import java.util.List;

public class RobotTankDrive extends TankDrive {

    public static final MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(RevRoboticsCoreHexMotor.class);
    public static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();
    private static final int TRACK_WIDTH = 1; /// tune this
    public static final double WHEEL_RADIUS = 1.77;
    public static final double GEAR_RATIO = 72;
    // public static final PIDCoefficients NORMAL_VELOCITY_PID = new PIDCoefficients(20, 8, 12); // TUNE THIS !!!!!!
    private BNO055IMU imu;


    private DriveConstraints constraints;
    private DcMotorEx bl, br, fr, fl;

    private HardwareMap hmap;
    private double poseHeading;
    private double offsetHeading;
    private boolean initialized = false;
    private Pose2d currPose;

    private TrajectoryFollower follower;

    public RobotTankDrive(final HardwareMap hmap) {
        super(TRACK_WIDTH);
        this.hmap = hmap;

        bl = (DcMotorEx) hmap.dcMotor.get("bl");
        br = (DcMotorEx) hmap.dcMotor.get("br");
        imu = hmap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        // fl = (DcMotorEx) hmap.dcMotor.get("fl");
        // fr = (DcMotorEx) hmap.dcMotor.get("fr");

        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.poseHeading = 0;
        this.offsetHeading = 0;

    }



    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();

        wheelPositions.add(encoderTicksToInches(bl.getCurrentPosition()));
        wheelPositions.add(encoderTicksToInches(br.getCurrentPosition()));
        //wheelPositions.add(encoderTicksToInches(fl.getCurrentPosition()));
        //wheelPositions.add(encoderTicksToInches(fr.getCurrentPosition()));

        return wheelPositions;
    }

    public static double encoderTicksToInches(final int ticks) {
        return 2 * 2 * Math.PI * 1.0 * ticks / TICKS_PER_REV;
    }

    @Override
    public void setMotorPowers(final
                                   double leftPow, final double rightPow){
        bl.setPower(leftPow);
        //fl.setPower(leftPow);
        br.setPower(rightPow);
        //fr.setPower(rightPow);
    }

    public void runWithEncoder(final double distance, final double power) {
        final int encoderDist = 1; // do something to change from inches to encoder tihngy
        setMotorPowers(power, power);
        fr.setTargetPosition(fr.getCurrentPosition() + encoderDist);
        fl.setTargetPosition(fl.getCurrentPosition() + encoderDist);
        br.setTargetPosition(br.getCurrentPosition() + encoderDist);
        bl.setTargetPosition(fr.getCurrentPosition() + encoderDist);
    }
    public void gamepadDrive(final double gamepadX, final double gamepadY) {
        bl.setPower(gamepadY + gamepadX);
        br.setPower(gamepadY - gamepadX);
        //fl.setPower(-gamepadY + gamepadX);
        //fr.setPower(-gamepadY - gamepadX);
    }

    private void setDriveConstraints(final double maxVeloctiy, final double maxAcceleration) {
        this.constraints = new DriveConstraints(maxVeloctiy, maxAcceleration, 0, 0);
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        bl.setPIDFCoefficients(runMode, coefficients);
        br.setPIDFCoefficients(runMode, coefficients);
    }

    public PIDFCoefficients getPIDFCoefficients(DcMotor.RunMode runMode) {
        return bl.getPIDFCoefficients(runMode);
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
    }

    public void updateFollower() {
        follower.update(getPoseEstimate());
    }

    public void update() {
        updatePoseEstimate();
        updateFollower();
    }

    public boolean isFollowingTrajectory() {
        return follower.isFollowing();
    }

    public BNO055IMU getIMU() {
        return imu;
    }



}
