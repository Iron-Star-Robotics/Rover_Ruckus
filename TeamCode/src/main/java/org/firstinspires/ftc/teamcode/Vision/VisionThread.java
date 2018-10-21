package org.firstinspires.ftc.teamcode.Vision;


import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Vision.GoldDetector;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.nio.ByteBuffer;

/** Vision thread to be run in parallel to the autonomous opMode
 * Created by Ben on 10/11
*/

public class VisionThread implements Runnable {
    public static int QUALITY = 25;
    public static double SCALE = 0.4;
    private VuforiaLocalizer.CloseableFrame frame;
    GoldDetector gd = new GoldDetector();
    FtcDashboard dashboard;

    public VisionThread(VuforiaLocalizer.CloseableFrame frame, FtcDashboard dashboard) {
        this.frame = frame;
        this.dashboard = dashboard;
    }


    @Override
    public void run() {
        for (int i = 0; i < this.frame.getNumImages(); i++) {
            Image image = this.frame.getImage(i);
            if (image.getFormat() == PIXEL_FORMAT.RGB565) {
                int imageWidth = image.getWidth(), imageHeight = image.getHeight();
                ByteBuffer byteBuffer = image.getPixels();

                Bitmap bmp = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
                bmp.copyPixelsFromBuffer(byteBuffer);
                Mat newMat = new Mat(bmp.getHeight(), bmp.getWidth(), CvType.CV_8UC3);
                Utils.bitmapToMat(bmp, newMat);
                Utils.matToBitmap(gd.process(newMat), bmp);

                this.dashboard.setImageQuality(QUALITY);
                this.dashboard.sendImage(bmp);
            }
        }

    }
}





