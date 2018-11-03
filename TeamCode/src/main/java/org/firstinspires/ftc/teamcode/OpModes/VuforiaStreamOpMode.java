package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;

import java.nio.ByteBuffer;
import java.util.concurrent.BlockingQueue;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
@Config
public class VuforiaStreamOpMode extends LinearOpMode {

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY = "YOUR KEY HERE";

    // adjust these parameters to suit your needs
    public static int MIN_LOOP_TIME = 100;
    public static int QUALITY = 25;
    public static double SCALE = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        FtcDashboard dashboard = FtcDashboard.getInstance();

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = "Aa7V0cb/////AAABmTsi2Ahmh0zWjzLxzU0mNG9cJ6TYhep67J8OK80SO/2+0+PwMBRgqCl9xE09y/Jp+8+zHWLX5onVDZfIpo1pVh7aivHulBtB7rXr5WLmzEWVRpPhNAnxRc1SVZ9H9cPmFRkIddugcPOc9W6+S+wHZGCg2Biv7Mvyao4uf8PJ8k5mNNJP1CWWE+4k86DkQjPGO8aFatDaDJOwTRQbr7niDGTcEdGU6pDbMWUaZItsMd7somOfiTZKMRSBn1ztRM/9uyzApzhLP4vB8cwHvD1RRdcK4D1Vk4Q6RrCIZRJd2jzfdoBzvAjora7yQAVMQkfwtpZnepTe29hRYLh8RXHt7fwDrWpZkMk74VNdkX7cM9mZ";

        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = vuforia.getFrameQueue();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (!frameQueue.isEmpty()) {
                long start = System.nanoTime();

                VuforiaLocalizer.CloseableFrame vuforiaFrame = null;
                try {
                    vuforiaFrame = frameQueue.take();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }

                if (vuforiaFrame == null) {
                    continue;
                }

                for (int i = 0; i < vuforiaFrame.getNumImages(); i++) {
                    Image image = vuforiaFrame.getImage(i);
                    if (image.getFormat() == PIXEL_FORMAT.RGB565) {
                        int imageWidth = image.getWidth(), imageHeight = image.getHeight();
                        ByteBuffer byteBuffer = image.getPixels();

                        Bitmap original = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
                        original.copyPixelsFromBuffer(byteBuffer);
                        Bitmap scaled = Bitmap.createScaledBitmap(original, (int) (SCALE * imageWidth), (int) (SCALE * imageHeight), false);

                        dashboard.setImageQuality(QUALITY);
                        dashboard.sendImage(scaled);
                    }
                }

                vuforiaFrame.close();

                long ms = (System.nanoTime() - start) / 1_000_000;
                long sleepTime = MIN_LOOP_TIME - ms;
                if (sleepTime > 0) {
                    Thread.sleep(sleepTime);
                }
            } else {
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }
}