package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.teamcode.Motion.DriveConstants;
import org.firstinspires.ftc.teamcode.Motion.Paths;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Vision.VisionService;
import org.opencv.core.Mat;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.concurrent.ExecutorService;
import java.util.zip.GZIPOutputStream;

@Config
public abstract class RRAuto extends LinearOpMode {
    protected Robot robot;
    protected GoldAlignDetector detector;

    protected enum GoldPos {
        LEFT,
        MIDDLE,
        RIGHT
    }

    protected enum Location {
        DEPOT,
        CRATER
    }

    Servo servo;

    protected GoldPos goldPos = null;

    DcMotorEx liftMotor;

    protected void init(boolean vision) {

        robot = new Robot(this);
        servo = hardwareMap.get(Servo.class, "marker");
        //robot.lift.raise();

        //robot.lift.delatch();
        liftMotor = hardwareMap.get(ExpansionHubMotor.class, "liftMotor");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(10);
        liftMotor.setPower(.8);
        while (!isStarted() && opModeIsActive()) {
            while (liftMotor.isBusy());
            liftMotor.setTargetPosition(10);
            liftMotor.setPower(.8);
        }


        if (vision) {
            detector = new GoldAlignDetector();
            detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

            detector.useDefaults();
            detector.alignSize = 200; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
            detector.alignPosOffset = 100; // How far from center frame to offset this alignment zone.
            detector.downscale = 0.4; // How much to downscale the input frames

            detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
            detector.maxAreaScorer.weight = 0.005; //

            detector.ratioScorer.weight = 5; //
            detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

            //visionService = new VisionService(detector);

        }
        detector.enable();


        robot.start();
        robot.drive.setBias(getBias());
    }

    public void sample() {

       // robot.drive.followFullTrajectory(Paths.centerDepotSample());
        if (detector.getAligned())
            goldPos = GoldPos.MIDDLE;
        else if (detector.isFound())
            goldPos = GoldPos.LEFT;
        else
            goldPos = GoldPos.RIGHT;

    }

    public void delatch() {
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(3400);
        liftMotor.setPower(1); // ik these calls r blocking but in this case I want to wait until they finish before performing any subsequent actions

        while(liftMotor.isBusy());
        liftMotor.setPower(0);

        if (getLocation() == Location.DEPOT)
            robot.drive.followFullTrajectory(Paths.unhookDepot());
        else
            robot.drive.followFullTrajectory(Paths.unhookLander());

        liftMotor.setTargetPosition(2000);
        liftMotor.setPower(1);
        while (liftMotor.isBusy());
        liftMotor.setPower(0);
        robot.drive.turnTo(Math.toDegrees(getBias()));

    }


    public abstract double getBias();
    public abstract Location getLocation();
    public abstract void runOpMode();
}
