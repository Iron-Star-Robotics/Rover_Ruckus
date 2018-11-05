package org.firstinspires.ftc.teamcode.Vision;


import com.disnodeteam.dogecv.OpenCVPipeline;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;

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

public class GoldDetector extends OpenCVPipeline { // here we are using the endercv module from DogeCv to make a cusotm detector

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
    public double alignPosOffset  = 0;    // How far from center frame is aligned
    public double alignSize       = 100;
    public enum MINERAL_POS {
        LEFT,
        CENTER,
        RIGHT
    }

    private MINERAL_POS mineralPos;

    public GoldDetector() {

    }

    public Mat process(Mat input) {
        input.copyTo(displayMat);
        input.copyTo(workingMat);
        input.release();


        //Preprocess the working Mat (blur it then apply a yellow filter)
        Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);
        yellowFilter.process(workingMat.clone(), yellowMask);

        //Find contours of the yellow mask and draw them to the display mat for viewing

        List<MatOfPoint> contoursYellow = new ArrayList<>();
        Imgproc.findContours(yellowMask, contoursYellow, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat,contoursYellow,-1,new Scalar(230,70,70),2);

        // Current result
        Rect bestRect = null;
        double maxArea = 0;

        // Loop through the contours and score them, searching for the best result
        for(MatOfPoint cont : contoursYellow){
            double area = VisionUtils.calculateArea(cont);
            Rect rect = Imgproc.boundingRect(cont);
            Imgproc.rectangle(workingMat, rect.tl(), rect.br(), new Scalar(0,0,255), 2);

            if (area > maxArea) {
                maxArea = area;
                bestRect = rect;
            }
        }

        // Vars to calculate the alignment logic.
        double alignX    = (getAdjustedSize().width / 2) + alignPosOffset; // Center point in X Pixels
        double alignXMin = alignX - (alignSize / 2); // Min X Pos in pixels
        double alignXMax = alignX +(alignSize / 2); // Max X pos in pixels
        double xPos; // Current Gold X Pos

        if(bestRect != null){
            // Show chosen result
            Imgproc.rectangle(displayMat, bestRect.tl(), bestRect.br(), new Scalar(255,0,0),4);
            Imgproc.putText(displayMat, "Chosen", bestRect.tl(),0,1,new Scalar(255,255,255));

            // Set align X pos
            xPos = bestRect.x + (bestRect.width / 2);
            goldXPos = xPos;

            // Draw center point
            Imgproc.circle(displayMat, new Point( xPos, bestRect.y + (bestRect.height / 2)), 5, new Scalar(0,255,0),2);

            // Check if the mineral is aligned
            if(xPos < alignXMax && xPos > alignXMin){
                mineralPos = MINERAL_POS.CENTER;
                aligned = true;
            }else{
                aligned = false;
            }

            if (!aligned) {
                if (xPos < alignXMin) {
                    mineralPos = MINERAL_POS.LEFT;
                } else {
                    mineralPos = MINERAL_POS.RIGHT;
                }
            }

            // Draw Current X
            Imgproc.putText(displayMat,"Current X: " + bestRect.x,new Point(10,getAdjustedSize().height - 10),0,0.5, new Scalar(255,255,255),1);
            found = true;
        }else{
            found = false;
            aligned = false;
        }

        //Print result
        Imgproc.putText(displayMat,"Result: " + aligned,new Point(10,getAdjustedSize().height - 30),0,1, new Scalar(255,255,0),1);


        return displayMat;

    }


    public boolean isAligned() {
        return aligned;
    }

    public Size getAdjustedSize() {
        return DOWNSCALED_SIZE;
    }

    public Mat processFrame(Mat rgba, Mat gray) {
        initSize = rgba.size();
        rgba.copyTo(workingMat);

        if(workingMat.empty()){
            return rgba;
        }

        Imgproc.resize(workingMat, workingMat,DOWNSCALED_SIZE); // Downscale
        Imgproc.resize(process(workingMat),workingMat,getInitSize()); // Process and scale back to original size for viewing
        //Print Info
        //Imgproc.putText(workingMat,"DogeCV 2018.2 " + detectorName + ": " + getAdjustedSize().toString() + " - " + speed.toString() ,new Point(5,30),0,0.5,new Scalar(0,255,255),2);

        return workingMat;
    }

    public Size getInitSize() {
        return initSize;
    }

    public MINERAL_POS getMineralPos() { return mineralPos; }







}
