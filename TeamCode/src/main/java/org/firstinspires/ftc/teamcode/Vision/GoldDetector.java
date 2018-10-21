package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
* Credit to DogeCV for these input values and the general alg :D!!!!!
* I just made changes to fit our vision needs and what I want to do with vision
 * Created by Ben on 10/7/18
*/

public class GoldDetector {

    private ColorFilter yellowFilter = new ColorFilter();
    private Mat workingMat = new Mat();
    private Mat yellowMask = new Mat();
    private Mat hiarchy = new Mat();
    private Mat displayMat = new Mat();
    private double goldXPos = 0; // x pos of the gold element
    private final int ALIGNMENT_OFFSET = 0; // how far from the center the frame is aligned
    private final int ALIGN_SIZE = 100; // how many pixels of error u want
    private boolean aligned = false;
    private boolean found = false;

    private Size initSize;
    private final Size DOWNSCALED_SIZE = new Size(640, 480);

    public Mat process(Mat input) {

        initSize = input.size();
        input.copyTo(workingMat);

        input.release();

        Imgproc.resize(workingMat, workingMat, DOWNSCALED_SIZE);

        Imgproc.GaussianBlur(workingMat, workingMat, new Size(5,5), 0);
        yellowFilter.process(workingMat.clone(), yellowMask);

        List<MatOfPoint> yellowContours = new ArrayList<>();
        Imgproc.findContours(yellowMask, yellowContours, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(workingMat, yellowContours, -1, new Scalar(230, 70,70));

        Rect bestRect = null;
        double maxArea = 0;
        for (MatOfPoint cont: yellowContours) {
            // Get bounding rectangle
            double area = VisionUtils.calculateArea(cont);
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(workingMat, rect.tl(), rect.br(), new Scalar(0,0,255), 2);

            if (area > maxArea) {
                maxArea = area;
                bestRect = rect;
            }

        }

        double alignX = (DOWNSCALED_SIZE.width / 2) + ALIGNMENT_OFFSET; // Center point in X Pixels
        double alignXMin = alignX - (ALIGN_SIZE / 2); // min xpos in pixel
        double alignXMax = alignX + (ALIGN_SIZE / 2);
        double xPos; // current gold xpos

        if (bestRect != null) {
            Imgproc.rectangle(displayMat, bestRect.tl(), bestRect.br(), new Scalar(255,0,0), 4);
            // Imgproc.putText(displayMat, )

            xPos = bestRect.x + (bestRect.width / 2);
            goldXPos = xPos;

            // Draw cetner point
            Imgproc.circle(displayMat, new Point(xPos, bestRect.y + (bestRect.height / 2)), 5, new Scalar(0,255,0), 2);

            // check if the mineral is aliged (DogeCV alg)
            if (xPos < alignXMax && xPos > alignXMin)
                aligned = true;
            else aligned = false;

            // Draw the current X
            Imgproc.putText(displayMat, "Current X: " + bestRect.x, new Point(10, DOWNSCALED_SIZE.height - 10), 0,0.5,new Scalar(255,255,255), 1);
            found = true;

        }
        else {
            aligned = false;
            found = false;
        }

        Imgproc.putText(displayMat, "result: " + aligned, new Point(10, DOWNSCALED_SIZE.height - 30), 0, 1, new Scalar(255,255,0), 1);

        return displayMat;

    }

    public boolean isAligned() {
        return aligned;
    }


}
