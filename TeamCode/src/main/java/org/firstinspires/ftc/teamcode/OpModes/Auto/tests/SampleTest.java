package org.firstinspires.ftc.teamcode.OpModes.Auto.tests;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.util.concurrent.ExecutorService;

public class SampleTest extends LinearOpMode {
    Robot robot;


    GoldAlignDetector detector;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.start();

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

        while (opModeIsActive()) {

        }

    }
}
