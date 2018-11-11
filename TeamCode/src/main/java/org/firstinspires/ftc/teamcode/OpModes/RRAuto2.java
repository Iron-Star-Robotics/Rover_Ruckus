package org.firstinspires.ftc.teamcode.OpModes;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.teamcode.Motion.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.util.concurrent.ExecutorService;

@Autonomous(name="RRAUTO2")
public class RRAuto2 extends LinearOpMode {
    private GoldAlignDetector detector;

    private ExecutorService frameConsumerExecutor;

    private double offsetHeading;
    private Robot robot;
    private ElapsedTime runtime = new ElapsedTime();
    // Temporary autonomous opmode using dogecv
    // We will replace this before the next qualifier with our custom vision api
    static final double     COUNTS_PER_MOTOR_REV    = DriveConstants.TICKS_PER_REV;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.3;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 200; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment


        frameConsumerExecutor = ThreadPool.newSingleThreadExecutor("frame stuff");
        frameConsumerExecutor.execute(() ->
                detector.enable()
        );

        robot = new Robot(this);
        robot.start();

        waitForStart();
        if (detector.getAligned()) {
            telemetry.log().add("straight aligned");
            encoderDrive(DRIVE_SPEED, -13, -13, 3);
            robot.drive.getServo().setPosition(-1.0);
        } else {
            encoderDrive(TURN_SPEED, -1, 1,3);
            if (!detector.getAligned()) {
                encoderDrive(TURN_SPEED, 2, -2, 3);
                encoderDrive(DRIVE_SPEED, -8, -8, 3);
                encoderDrive(TURN_SPEED, -1.2,1.2, 3);
                encoderDrive(DRIVE_SPEED, -4, -4, 2);
                robot.drive.getServo().setPosition(-1.0);


                encoderDrive(DRIVE_SPEED, -1, -1, 2);

            } else {
                telemetry.log().add("left aligned");
                encoderDrive(DRIVE_SPEED, -7, -7, 3);
                encoderDrive(TURN_SPEED, 1.2,-1.2, 3);
                encoderDrive(DRIVE_SPEED, -4, -4, 2);
                encoderDrive(TURN_SPEED, 1.2, -1.2, 2);
                robot.drive.getServo().setPosition(-1.0);
                encoderDrive(DRIVE_SPEED, -1, -1, 2);

            }
        }

        while(opModeIsActive()) {

        }

        detector.disable();
    }

    /*
     * Code to run REPEATEDLY when the driver hits INIT


    /*
     * Code to run REPEATEDLY when the driver hits PLAY
     */


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        DcMotorEx leftDrive = robot.drive.getMotors().get(0);
        DcMotorEx rightDrive = robot.drive.getMotors().get(1);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}
