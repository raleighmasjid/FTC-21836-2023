package org.firstinspires.ftc.teamcode.control.vision;

import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.CENTER;
import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.LEFT;
import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.RIGHT;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRed;

import static java.lang.Math.max;
import static java.lang.Math.round;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PropDetectPipeline extends OpenCvPipeline {

    private final Telemetry telemetry;
    private final Mat mat = new Mat();

    public enum Randomization {
        LEFT(1, 2),
        CENTER(3, 4),
        RIGHT(6, 5);

        Randomization(int x1, int x2) {
            this.x1 = x1;
            this.x2 = x2;
        }

        public final int x1, x2;
        public static final Randomization[] randomizations = values();
    }

    private Randomization location;

    public static double
            X_LEFT_BOUND = 0,
            X_LEFT_CENTER_BOUND = 213.3,
            X_CENTER_RIGHT_BOUND = 426.6,
            X_RIGHT_BOUND = 640,
            Y_TOP = 0,
            Y_BOTTOM = 480;

    public static HSV
            minBlue = new HSV(
                    90,
                    75,
                    5
            ),
            maxBlue = new HSV(
                    150,
                    255,
                    255
            ),
            minRed = new HSV(
                    0,
                    75,
                    5
            ),
            maxRed = new HSV(
                    0,
                    255,
                    255
            );

    private final Rect
            LEFT_AREA = new Rect(
                    new Point(X_LEFT_BOUND, Y_TOP),
                    new Point(X_LEFT_CENTER_BOUND, Y_BOTTOM)
            ),
            CENTER_AREA = new Rect(
                    new Point(X_LEFT_CENTER_BOUND, Y_TOP),
                    new Point(X_CENTER_RIGHT_BOUND, Y_BOTTOM)
            ),
            RIGHT_AREA = new Rect(
                    new Point(X_CENTER_RIGHT_BOUND, Y_TOP),
                    new Point(X_RIGHT_BOUND, Y_BOTTOM)
            );

    public PropDetectPipeline(Telemetry t) {
        telemetry = t;
    }

    public Randomization getLocation() {
        return location;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Executed every time a new frame is dispatched
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(mat, (isRed ? minRed : minBlue).toScalar(), (isRed ? maxRed : maxBlue).toScalar(), mat);

        Mat left = mat.submat(LEFT_AREA);
        Mat middle = mat.submat(CENTER_AREA);
        Mat right = mat.submat(RIGHT_AREA);

        double leftValue = Core.sumElems(left).val[0] / LEFT_AREA.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / CENTER_AREA.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_AREA.area() / 255;

        int leftInt = (int) round(leftValue * 100);
        int middleInt = (int) round(middleValue * 100);
        int rightInt = (int) round(rightValue * 100);

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

        Imgproc.rectangle(mat, LEFT_AREA, location == LEFT ? colorFound : colorDefault);
        Imgproc.rectangle(mat, CENTER_AREA, location == CENTER ? colorFound : colorDefault);
        Imgproc.rectangle(mat, RIGHT_AREA, location == RIGHT ? colorFound : colorDefault);

        return mat;
    }
}