package org.firstinspires.ftc.teamcode.control.vision;

import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.CENTER;
import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.LEFT;
import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.RIGHT;

import static java.lang.Math.max;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PropDetectPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Randomization {
        LEFT,
        CENTER,
        RIGHT;
    }

    private Randomization location;

    private static final Rect LEFT_ROI = new Rect(
            new Point(0, 0),
            new Point(213.3, 480)
    );
    private static final Rect CENTER_ROI = new Rect(
            new Point(213.3, 0),
            new Point(426.6, 480)
    );
    private static final Rect RIGHT_ROI = new Rect(
            new Point(426.6, 0),
            new Point(640, 480)
    );

    private Scalar
            minColor = new Scalar(90, 75, 5),
            maxColor = new Scalar(150, 255, 255);

    public PropDetectPipeline(Telemetry t) {
        telemetry = t;
    }

    public void setColorBounds(Scalar minColor, Scalar maxColor) {
        this.minColor = minColor;
        this.maxColor = maxColor;
    }

    public Randomization getLocation() {
        return location;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Executed every time a new frame is dispatched
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(mat, minColor, maxColor, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat middle = mat.submat(CENTER_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / CENTER_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        int leftInt = (int) Math.round(leftValue * 100);
        int middleInt = (int) Math.round(middleValue * 100);
        int rightInt = (int) Math.round(rightValue * 100);

        left.release();
        middle.release();
        right.release();

        telemetry.addData("Left", leftInt + "%");
        telemetry.addData("Center", middleInt + "%");
        telemetry.addData("Right", rightInt + "%");

        int max = max(leftInt, max(rightInt, middleInt));

        location = max == rightInt ? RIGHT : max == leftInt ? LEFT : CENTER;
        telemetry.addData("Prop Location", location.toString());
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorDefault = new Scalar(0, 0, 0);
        Scalar colorFound = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == LEFT ? colorFound : colorDefault);
        Imgproc.rectangle(mat, CENTER_ROI, location == CENTER ? colorFound : colorDefault);
        Imgproc.rectangle(mat, RIGHT_ROI, location == RIGHT ? colorFound : colorDefault);

        return mat;
    }
}