package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;
import android.widget.FrameLayout;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ThreadPool;
import com.qualcomm.robotcore.util.Util;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.nio.ByteBuffer;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;

import static org.firstinspires.ftc.teamcode.OpModes.VuforiaStreamOpMode.VUFORIA_LICENSE_KEY;

public class VuforiaCamera {
    private VuforiaLocalizer vuforia;
    private FrameLayout cameraLayout;
    private ExecutorService frameConsumerExecutor;
    private GoldDetector detector;
    private final double SCALE = 0.4;
    private final int QUALITY = 25;

    private class FrameConsumer implements Runnable {
        private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;
        private Mat frame;
        private byte[] frameBuffer;
        private FtcDashboard dashboard;


        private FrameConsumer(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue, FtcDashboard dashboard) {
            this.frameQueue = frameQueue;
        }

        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                // get frames and process them
                if (!frameQueue.isEmpty()) {
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
                            Utils.bitmapToMat(original, this.frame);
                            Bitmap scaled = Bitmap.createScaledBitmap(original, (int) (SCALE * imageWidth), (int) (SCALE * imageHeight), false);
                            Utils.matToBitmap(onFrame(this.frame), scaled);

                            dashboard.setImageQuality(QUALITY);
                            dashboard.sendImage(scaled);
                        }
                    }
                    vuforiaFrame.close();


                }


            }

        }

    }

    private Mat onFrame(Mat frame) {
        return detector.process(frame);
    }

    public void init(FtcDashboard dashboard) {
        detector = new GoldDetector();
        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);
        vuforia.setFrameQueueCapacity(1);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        frameConsumerExecutor = ThreadPool.newSingleThreadExecutor("Vuforia frame consumer");
        frameConsumerExecutor.execute(new FrameConsumer(vuforia.getFrameQueue(), dashboard));
    }

    public void close() {
        if (frameConsumerExecutor != null) {
            frameConsumerExecutor.shutdownNow();
            frameConsumerExecutor = null;
        }
    }


}
