package org.firstinspires.ftc.teamcode.Vision;
import android.graphics.Bitmap;
import android.support.annotation.Nullable;
import android.util.Log;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class VisionUtils {
    /**
     *
     * Static class containing utilities for image processing
     * Created by Ben on 10/9/18
     *
     */

    public static Mat bitmapToMat(Bitmap bmp, int cvType) {
        Mat newMat = new Mat(bmp.getHeight(), bmp.getWidth(), cvType);
        Utils.bitmapToMat(bmp, newMat);

        return newMat;
    }

    public static Mat frameToMatrix(VuforiaLocalizer.CloseableFrame frame) {
        Image img = frame.getImage(0);
        Bitmap bmp = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
        bmp.copyPixelsFromBuffer(img.getPixels());

        return bitmapToMat(bmp, CvType.CV_8UC3);
    }


    public static double calculateArea(Mat input) {
        MatOfPoint contour = (MatOfPoint) input;
        double area = Imgproc.contourArea(contour);

        return -area;
    }






}
