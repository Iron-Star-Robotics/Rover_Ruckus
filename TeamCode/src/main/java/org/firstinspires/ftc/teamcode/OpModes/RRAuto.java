package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.RobotTankDriveBase;
import org.firstinspires.ftc.teamcode.Subsystems.RobotTankDriveOptimized;

import java.util.concurrent.ExecutorService;


@TeleOp(name="GoldAlign Example", group="DogeCV")

public class RRAuto extends OpMode
{
    // Detector object
    private GoldAlignDetector detector;

    private ExecutorService frameConsumerExecutor;

    private double offsetHeading;
    private Robot robot;
    // Temporary autonomous opmode using dogecv
    // We will replace this before the next qualifier with our custom vision api


    @Override
    public void init() {
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment


        frameConsumerExecutor = ThreadPool.newSingleThreadExecutor("frame stuff");
        frameConsumerExecutor.execute(new Runnable() {
            @Override
            public void run() {
                detector.enable();
            }
        });

        robot = new Robot(this);
        robot.start();

    }

    /*
     * Code to run REPEATEDLY when the driver hits INIT
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // lower the lift
        offsetHeading = robot.drive.getExternalHeading();

    }

    /*
     * Code to run REPEATEDLY when the driver hits PLAY
     */
    @Override
    public void loop() {
        telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
        telemetry.addData("X Pos" , detector.getXPosition()); // Gold X position.

        if (detector.getAligned()) {
            robot.drive.robotLine(new Vector2d(-36, 36));
        } else {
            robot.drive.robotTurn(Math.PI / 4);
            if (detector.getAligned()) {
                robot.drive.robotLine(new Vector2d(-25, 46));
            } else {
                robot.drive.robotTurn(Math.PI / 2);
                robot.drive.robotLine(new Vector2d(-46, 25));
            }
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Disable the detector
        detector.disable();
    }

}