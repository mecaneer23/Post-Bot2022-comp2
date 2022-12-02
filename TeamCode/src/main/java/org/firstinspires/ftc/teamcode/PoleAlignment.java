package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PoleAlignment extends OpenCvPipeline {
    private volatile goDirection direction = goDirection.NEITHER;

    // might need to change order
//    private List<Point> ideal = Arrays.asList(
//            new Point(100, 0),
//            new Point(200, 0),
//            new Point(100, 240),
//            new Point(200, 240)
//    );
//
//    private final Mat gray = new Mat();
//    private final Mat blur = new Mat();
//    private final Mat edges = new Mat();
//    private final List<MatOfPoint> contours = new ArrayList<>();
//    private final List<Point> compare = new ArrayList<>();

    private static final int
            X = 130,
            Y = 168,
            W = 30,
            H = 50;
    private static final Scalar
            lower_yellow_bounds = new Scalar(100, 100, 0, 255),
            upper_yellow_bounds = new Scalar(255, 255, 200, 255);
    private final Scalar
            YELLOW = new Scalar(255, 255, 0);
    private final Point
            sleeve_pointA = new Point(X, Y),
            sleeve_pointB = new Point(X + W, Y + H);
    private double yelPercent;
    private final Mat yelMat = new Mat();
    private Mat blurredMat = new Mat();
    private Mat kernel = new Mat();

//    @Override
//    public Mat processFrame(Mat input) {
//        Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
//        Imgproc.GaussianBlur(gray, blur, new Size(3, 3), 0);
//
//        Imgproc.Canny(blur, edges, 100, 200);
//        Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//
//        Converters.Mat_to_vector_Point(contours.get(0), compare);
//        if (compare.get(0).x + 10 < ideal.get(0).x) {
//            direction = goDirection.LEFT;
//        } else if (compare.get(0).x - 10 > ideal.get(0).x) {
//            direction = goDirection.RIGHT;
//        } else {
//            direction = goDirection.NEITHER;
//        }
//
//        return input;
//    }


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        blurredMat = blurredMat.submat(new Rect(sleeve_pointA, sleeve_pointB));

        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

        Core.inRange(blurredMat, lower_yellow_bounds, upper_yellow_bounds, yelMat);

        yelPercent = Core.countNonZero(yelMat);

        double maxPercent = Math.max(yelPercent, yelMat.total());

        if (maxPercent == yelPercent) {
            direction = goDirection.NEITHER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
        }

        blurredMat.release();
        yelMat.release();
        kernel.release();

        return input;
    }

    public goDirection getDirection() {
        return direction;
    }

    public enum goDirection {
        NEITHER, // within margin of error
        LEFT,
        RIGHT
    }
}
