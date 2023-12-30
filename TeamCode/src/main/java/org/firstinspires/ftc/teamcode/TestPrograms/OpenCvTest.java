package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.CvType;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.List;

@Disabled
public class OpenCvTest {

    private static final Scalar LOWER_COLOR = new Scalar(20, 100, 100);
    private static final Scalar UPPER_COLOR = new Scalar(30,255,255);

    private static final int IMAGE_CENTER_X = 320;
    private static final int THRESHOLD = 50;

    public int detectConePosition(Mat frame){

        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_BGR2HSV);

        Core.inRange(frame, LOWER_COLOR, UPPER_COLOR, frame);

        Mat hierarchy = new Mat();
        MatOfPoint contours = new MatOfPoint();
        Imgproc.findContours(frame, (List<MatOfPoint>) contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int location = 2;

        for(MatOfPoint contour: (List<MatOfPoint>) contours){

            MatOfPoint2f points = new MatOfPoint2f(contour.toArray());
            Moments moments = Imgproc.moments(points);
            double cx = moments.m10 / moments.m00;
            double cy = moments.m01 / moments.m00;

            if(cx < IMAGE_CENTER_X - THRESHOLD){
                location = 1;
            } else if(cx > IMAGE_CENTER_X + THRESHOLD){
                location = 3;
            }
        }

        hierarchy.release();

        return location;

    }
}
