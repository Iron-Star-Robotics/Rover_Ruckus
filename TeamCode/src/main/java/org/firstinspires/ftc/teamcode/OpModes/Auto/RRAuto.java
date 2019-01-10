package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.teamcode.Motion.Paths;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.util.concurrent.ExecutorService;

@Config
public abstract class RRAuto extends LinearOpMode {
    protected Robot robot;
    protected GoldAlignDetector detector;
    double imageOffset = 100;
    public static double turnDeg = 30;
    public static double strafeDist = 10;
    public static double forwardDist = 20;

    protected void init(boolean vision) {

        robot = new Robot(this);
        robot.lift.raise();
        robot.drive.strafeLeft(strafeDist);

        if (vision) {
            detector = new GoldAlignDetector();
            detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

            detector.useDefaults();
            detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
            detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
            detector.downscale = 0.4; // How much to downscale the input frames

            detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
            detector.maxAreaScorer.weight = 0.005; //

            detector.ratioScorer.weight = 5; //
            detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

            ExecutorService visionExecutor = ThreadPool.newSingleThreadExecutor("visionExecutor");

            visionExecutor.submit(() -> detector.enable()); // Start the detector!

            if (detector.getXPosition() < 320 - imageOffset)
                robot.drive.turnTo(-turnDeg);
            else if (!detector.getAligned())
                robot.drive.turnTo(turnDeg);
            robot.drive.forward(forwardDist);

            detector.disable();
        }
    }

    public abstract void runOpMode();
}
