package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/*
* Created by Ben
 **/

public class ColorFilter {
    private List<Mat> channels;
    private final int YELLOW_THRESHOLD = 70;

    // Add a contructer if we ever want to do multiple colors or something idk

    public void process(Mat input, Mat mask) {
        channels = new ArrayList<>();

        // Yellow filter

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YUV);
        Imgproc.GaussianBlur(input, input, new Size(3,3), 0);
        Core.split(input, channels);
        Imgproc.threshold(channels.get(1), mask, YELLOW_THRESHOLD, 255, Imgproc.THRESH_BINARY); // get the green channel and set its max to 255

        for (int i = 0; i < channels.size(); i++)
            channels.get(i).release();
    }
}
