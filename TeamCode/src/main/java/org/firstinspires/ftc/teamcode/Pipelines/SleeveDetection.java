package org.firstinspires.ftc.teamcode.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SleeveDetection extends OpenCvPipeline {
    private static final int
        X = 205,
        Y = 150,
        W = 20,
        H = 50;
    private static final Scalar
            lower_yellow_bounds = new Scalar(100, 100, 0, 255),
            upper_yellow_bounds = new Scalar(255, 255, 200, 255),
            lower_cyan_bounds = new Scalar(0, 100, 100, 255),
            upper_cyan_bounds = new Scalar(200, 255, 255, 255),
            lower_magenta_bounds = new Scalar(50, 0, 50, 255),
            upper_magenta_bounds = new Scalar(255, 200, 255, 255);
    private final Scalar
            YELLOW = new Scalar(255, 255, 0),
            CYAN = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255),
            WHITE = new Scalar(255, 255, 255);
    private final Point
            sleeve_pointA = new Point(X, Y),
            sleeve_pointB = new Point(X + W, Y + H);
    private double yelPercent, cyaPercent, magPercent;
    private final Mat yelMat = new Mat();
    private final Mat cyaMat = new Mat();
    private final Mat magMat = new Mat();
    private Mat blurredMat = new Mat();
    private Mat kernel = new Mat();
    private volatile ParkingPosition position = ParkingPosition.DEFAULT;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.blur(input, blurredMat, new Size(5, 5));
        blurredMat = blurredMat.submat(new Rect(sleeve_pointA, sleeve_pointB));

        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(blurredMat, blurredMat, Imgproc.MORPH_CLOSE, kernel);

        Core.inRange(blurredMat, lower_yellow_bounds, upper_yellow_bounds, yelMat);
        Core.inRange(blurredMat, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
        Core.inRange(blurredMat, lower_magenta_bounds, upper_magenta_bounds, magMat);

        yelPercent = Core.countNonZero(yelMat);
        cyaPercent = Core.countNonZero(cyaMat);
        magPercent = Core.countNonZero(magMat);

        double maxPercent = Math.max(yelPercent, Math.max(cyaPercent, magPercent));

        if (maxPercent == yelPercent) {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
        } else if (maxPercent == cyaPercent) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    CYAN,
                    2
            );
        } else if (maxPercent == magPercent) {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    MAGENTA,
                    2
            );
        } else {
            position = ParkingPosition.DEFAULT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    WHITE,
                    2
            );
        }

        blurredMat.release();
        yelMat.release();
        cyaMat.release();
        magMat.release();
        kernel.release();

        return input;
    }

    public ParkingPosition getPosition() {
        return position;
    }

    public enum ParkingPosition {
        DEFAULT,
        LEFT,
        CENTER,
        RIGHT
    }
}