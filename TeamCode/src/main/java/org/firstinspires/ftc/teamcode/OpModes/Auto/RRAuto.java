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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.teamcode.Motion.DriveConstants;
import org.firstinspires.ftc.teamcode.Motion.Paths;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Vision.VisionService;
import org.opencv.core.Mat;

import java.util.concurrent.ExecutorService;
import java.util.zip.GZIPOutputStream;

@Config
public abstract class RRAuto extends LinearOpMode {
    protected Robot robot;
    protected GoldAlignDetector detector;


    private VisionService.GoldPos goldPos;

    VisionService visionService;
    double imageOffset = 50;
    public static double turnDeg = 20;
    public static double strafeDist = 10;
    public static double forwardDist = 20;

    Servo servo;

    protected void init(boolean vision) {

        robot = new Robot(this);
        servo = hardwareMap.get(Servo.class, "marker");
        //robot.lift.raise();

        //robot.lift.delatch();

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
    }

    public void sample() {


       // goldPos = visionService.getGoldLocation();
        //visionService.stop();
        String goldDisplay;
        if (detector.getAligned()) {
            goldPos = VisionService.GoldPos.CENTER;
            goldDisplay = "Center";
            Trajectory trajectory = centerTrajectory();
            robot.drive.setTrajectory(trajectory);
            robot.drive.followTrajectory(trajectory);
            while (opModeIsActive() && robot.drive.isFollowingTrajectory()) {

            }
            telemetry.log().add("Gold pos: " + goldDisplay);
        }
        else if (detector.isFound() && !detector.getAligned()) {
            goldPos = VisionService.GoldPos.LEFT;
            goldDisplay = "Left";
            Trajectory trajectory = leftSampleTrajectory();
            robot.drive.setTrajectory(trajectory);
            robot.drive.followTrajectory(trajectory);

            while (opModeIsActive() && robot.drive.isFollowingTrajectory()) {

            }
            telemetry.log().add("Gold pos: " + goldDisplay);

        }
        else {
            goldPos = VisionService.GoldPos.RIGHT;
            goldDisplay = "Right";
            Trajectory trajectory = rightSampleTrajectory();
            robot.drive.setTrajectory(trajectory);
            robot.drive.followTrajectory(trajectory);
            while (opModeIsActive() && robot.drive.isFollowingTrajectory()) {

            }
            telemetry.log().add("Gold pos: " + goldDisplay);

        }

        sleep(200);

        telemetry.log().add("Gold pos: " + goldDisplay);
        detector.disable();

        robot.drive.setBias(Math.PI / 4);

    }

    public Trajectory rightSampleTrajectory() {
        return robot.drive.trajectoryBuilder(new Pose2d(14.183, 14.183, Math.PI / 4))
                .strafeLeft(7)
                .waitFor(1)
                .turnTo(0)
                .waitFor(1)
                .lineTo(new Vector2d(45,25), new ConstantInterpolator(0))
                .waitFor(1)
                .lineTo(new Vector2d(24,24), new ConstantInterpolator(0))
                .waitFor(1)
                .turnTo(Math.toRadians(115))
                .build();
    }

    public Trajectory leftSampleTrajectory() {
        return robot.drive.trajectoryBuilder(new Pose2d(14.183, 14.183, Math.PI / 4))
                .strafeLeft(7)
                .waitFor(1)
                .turnTo(Math.toRadians(65))
                .waitFor(1)
                .lineTo(new Vector2d(25,45), new ConstantInterpolator(Math.toRadians(65)))
                .waitFor(1)
                .back(10)
                .waitFor(1)
                .turnTo(Math.toRadians(135))
                .build();
    }

    public Trajectory centerTrajectory() {
        return robot.drive.trajectoryBuilder(new Pose2d(14.183, 14.183, Math.PI / 4))
                .strafeLeft(7)
                .waitFor(1)
                .lineTo(new Vector2d(33,33), new ConstantInterpolator(Math.PI / 4))
                .waitFor(1)
                .lineTo(new Vector2d(24,24), new ConstantInterpolator(Math.PI / 4))
                .waitFor(1)
                .turnTo(Math.toRadians(125))
                .build();
    }

    public abstract void runOpMode();
}
