package org.firstinspires.ftc.teamcode.Vision;

/*import android.app.Activity;
import android.graphics.Bitmap;
import android.widget.FrameLayout;
import android.widget.LinearLayout;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ThreadPool;
import com.qualcomm.robotcore.util.Util;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.nio.ByteBuffer;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;


public class VuforiaCamera {
    private VuforiaLocalizer vuforia;
    private FrameLayout cameraLayout;
    private ExecutorService frameConsumerExecutor;
    private GoldDetector detector;
    private final double SCALE = 0.4;
    private final int QUALITY = 25;
    private static String VUFORIA_LICENSE_KEY = "Aa7V0cb/////AAABmTsi2Ahmh0zWjzLxzU0mNG9cJ6TYhep67J8OK80SO/2+0+PwMBRgqCl9xE09y/Jp+8+zHWLX5onVDZfIpo1pVh7aivHulBtB7rXr5WLmzEWVRpPhNAnxRc1SVZ9H9cPmFRkIddugcPOc9W6+S+wHZGCg2Biv7Mvyao4uf8PJ8k5mNNJP1CWWE+4k86DkQjPGO8aFatDaDJOwTRQbr7niDGTcEdGU6pDbMWUaZItsMd7somOfiTZKMRSBn1ztRM/9uyzApzhLP4vB8cwHvD1RRdcK4D1Vk4Q6RrCIZRJd2jzfdoBzvAjora7yQAVMQkfwtpZnepTe29hRYLh8RXHt7fwDrWpZkMk74VNdkX7cM9mZ";

    private class FrameConsumer implements Runnable {
        private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;
        private Mat frame;
        private FtcDashboard dashboard;


        private FrameConsumer(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue, FtcDashboard dashboard) {
            this.frameQueue = frameQueue;
            this.dashboard = dashboard;
        }

        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                //get frames and process them

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
                            if (this.frame == null) {
                                this.frame = new Mat(imageHeight, imageWidth, CvType.CV_8UC3);
                            }
                            ByteBuffer byteBuffer = image.getPixels();
                            Bitmap original = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
                            original.copyPixelsFromBuffer(byteBuffer);
                            Utils.bitmapToMat(original, this.frame);
                            Utils.matToBitmap(onFrame(this.frame), original);
                            Bitmap scaled = Bitmap.createScaledBitmap(original, (int) (SCALE * imageWidth), (int) (SCALE * imageHeight), false);
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
            return detector.processFrame(frame, null);
        }

        public void init(FtcDashboard dashboard, HardwareMap hmap) {
            detector = new GoldDetector();
            detector.init(hmap.appContext, CameraViewDisplay.getInstance());
            detector.enable();
            /*
            VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
            vuforiaParams.vuforiaLicenseKey = "Aa7V0cb/////AAABmTsi2Ahmh0zWjzLxzU0mNG9cJ6TYhep67J8OK80SO/2+0+PwMBRgqCl9xE09y/Jp+8+zHWLX5onVDZfIpo1pVh7aivHulBtB7rXr5WLmzEWVRpPhNAnxRc1SVZ9H9cPmFRkIddugcPOc9W6+S+wHZGCg2Biv7Mvyao4uf8PJ8k5mNNJP1CWWE+4k86DkQjPGO8aFatDaDJOwTRQbr7niDGTcEdGU6pDbMWUaZItsMd7somOfiTZKMRSBn1ztRM/9uyzApzhLP4vB8cwHvD1RRdcK4D1Vk4Q6RrCIZRJd2jzfdoBzvAjora7yQAVMQkfwtpZnepTe29hRYLh8RXHt7fwDrWpZkMk74VNdkX7cM9mZ";

            vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

            Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
            vuforia.setFrameQueueCapacity(1);

            frameConsumerExecutor = ThreadPool.newSingleThreadExecutor("Vuforia frame consumer");
            frameConsumerExecutor.execute(new Runnable() {
                @Override
                public void run() {
                    detector.enable();
                }
            });
        }

        private void close() {
            if (frameConsumerExecutor != null) {
                frameConsumerExecutor.shutdownNow();
                frameConsumerExecutor = null;
            }

    }


}

*/