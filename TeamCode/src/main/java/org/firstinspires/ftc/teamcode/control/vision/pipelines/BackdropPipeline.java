/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.control.vision.pipelines;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.black;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.blue;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.draw3dCubeMarker;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.drawAxisMarker;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.gray;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.green;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.lavender;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.poseFromTrapezoid;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.red;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.white;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.yellow;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.GREEN;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.INVALID;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.PURPLE;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.WHITE;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.YELLOW;
import static java.lang.Math.round;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class BackdropPipeline extends OpenCvPipeline {

    public static final double SCREEN_HEIGHT = 1280, SCREEN_WIDTH = 960;

    public static final Point
            CORNER_TL = new Point(0, 0),
            CORNER_TR = new Point(SCREEN_WIDTH, 0),
            CORNER_BR = new Point(SCREEN_WIDTH, SCREEN_HEIGHT),
            CORNER_BL = new Point(0, SCREEN_HEIGHT);

    public boolean
            warp = false,
            backdropVisible = false,
            isRed = true,
            editPoints = true,
            graphic = true,
            background = false,
            refresh = true;

    public double
            X_TOP_LEFT_R_TAG = 16.5,
            Y_TOP_LEFT = 32.42857142857142,
            TARGET_SIZE = 65,
            X_SHIFT_L_TAG_TO_L_PIXEL = -2.9,
            Y_SHIFT_TAG_TO_PIXEL = -2.6,
            X_SHIFT_PIXEL_POINTS_R = 1.8857142857142857,
            Y_SHIFT_PIXEL_POINTS_T = -1.1428571428571428,
            Y_SHIFT_PIXEL_POINTS_B = 0.7428571428571429,
            X_SHIFT_WHITE = 0.08,
            Y_SHIFT_WHITE = 2.5,
            X_SHIFT_BLACK = 5,
            Y_SHIFT_BLACK = -3.0,
            X_SHIFT_U = 5.2,
            Y_SHIFT_U = -30.5;

    private static final double[]
            minPurple = {140, .15, .3},
            maxPurple = {320, 1, 1},

            minYellow = {20, .51, .5},
            maxYellow = {50, 1, 1},

            minGreen =  {70, .4, .215},
            maxGreen =  {110, 1, 1},

            minWhite =  {0, 0, 0.54},
            maxWhite =  {360, 0.249, 1};

    private final ArrayList<AprilTagDetection> tags = new ArrayList<>();

    private long nativeApriltagPtr;
    private final Mat grey = new Mat(), cameraMatrix;

    public final Backdrop backdrop = new Backdrop();
    private final Point[][][] points = new Point[11][7][4];

    private final double
            fx = 1430,
            fy = 1430,
            cx = 480,
            cy = 620;

    public BackdropPipeline(Telemetry telemetry) {

        generatePoints();

        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);

        cameraMatrix.put(0, 0, fx);
        cameraMatrix.put(0, 1, 0);
        cameraMatrix.put(0, 2, cx);

        cameraMatrix.put(1, 0, 0);
        cameraMatrix.put(1, 1, fy);
        cameraMatrix.put(1, 2, cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2, 1, 0);
        cameraMatrix.put(2, 2, 1);

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
        this.telemetry = telemetry;
    }

    private final Telemetry telemetry;

    @Override
    protected void finalize() {
        // Might be null if createApriltagDetector() threw an exception
        if (nativeApriltagPtr != 0) {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        } else {
            System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        // Convert to greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        double tagSize = 0.0508;

        // Run AprilTag
        ArrayList<AprilTagDetection> detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagSize, fx, fy, cx, cy);

        // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
        // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
        for (AprilTagDetection detection : detections) {
            AprilTagDetectionPipeline.Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagSize, tagSize);
            drawAxisMarker(input, tagSize / 2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
            draw3dCubeMarker(input, tagSize, tagSize, tagSize, 5, pose.rvec, pose.tvec, cameraMatrix);
        }
        grey.release();

        tags.clear();

        int left = !isRed ? 1 : 4;
        int middle = !isRed ? 2 : 5;
        int right = !isRed ? 3 : 6;

        for (AprilTagDetection detection : detections) {
            int id = detection.id;
            if (id == left || id == middle || id == right) {
                tags.add(detection);
            }
        }

        backdropVisible = !tags.isEmpty();

        if (backdropVisible) {

            int minInd = 0, maxInd = 0;
            for (int i = 0; i < tags.size(); i++) {
                if (tags.get(i).id < tags.get(minInd).id) minInd = i;
                if (tags.get(i).id > tags.get(maxInd).id) maxInd = i;
                if (tags.size() < 3 && tags.get(i).id % 3 == 2) {
                    minInd = maxInd = i;
                    break;
                }
            }

            Point bl = tags.get(minInd).corners[0];
            Point tl = tags.get(minInd).corners[3];
            Point br = tags.get(maxInd).corners[1];
            Point tr = tags.get(maxInd).corners[2];

            MatOfPoint2f srcTag = new MatOfPoint2f(
                    bl,
                    br,
                    tr,
                    tl
            );

            double leftX = getLeftX(tags.get(minInd).id);
            double rightX = getLeftX(tags.get(maxInd).id) + TARGET_SIZE;

            Point
                    tagTR = new Point(rightX, (Y_TOP_LEFT * TARGET_SIZE / 2.0)),
                    tagBL = new Point(leftX, (Y_TOP_LEFT * TARGET_SIZE / 2.0) + TARGET_SIZE),
                    tagTL = new Point(leftX, (Y_TOP_LEFT * TARGET_SIZE / 2.0)),
                    tagBR = new Point(rightX, (Y_TOP_LEFT * TARGET_SIZE / 2.0) + TARGET_SIZE);

            MatOfPoint2f dstTag = new MatOfPoint2f(
                    tagBL,
                    tagBR,
                    tagTR,
                    tagTL
            );

            Imgproc.line(input, tl, tr, blue, 3);
            Imgproc.line(input, bl, br, blue, 3);
            Imgproc.line(input, tl, bl, blue, 3);
            Imgproc.line(input, tr, br, blue, 3);

            Mat transformMatrix = Imgproc.getPerspectiveTransform(srcTag, dstTag);
            Imgproc.warpPerspective(input, input, transformMatrix, input.size());
            transformMatrix.release();

            if (editPoints) generatePoints();

            double size = 5;
            for (Point[][] row : points) for (Point[] pair : row) for (Point point : pair) {
                if (point == null) continue;
                Imgproc.rectangle(
                        input,
                        new Point(point.x - size, point.y - size),
                        new Point(point.x + size, point.y + size),
                        blue,
                        2
                );
            }

            Point whiteSample = new Point(
                    getLeftX(tags.get(0).id) + (X_SHIFT_WHITE * TARGET_SIZE / 2.0),
                    (Y_TOP_LEFT * TARGET_SIZE / 2.0) + (Y_SHIFT_WHITE * TARGET_SIZE / 2.0)
            );
            telemetry.addData("first tag id", getLeftX(tags.get(0).id));
            Point blackSample = new Point(
                    (X_TOP_LEFT_R_TAG * TARGET_SIZE / 2.0) + (X_SHIFT_BLACK * TARGET_SIZE / 2.0),
                    (Y_TOP_LEFT * TARGET_SIZE / 2.0) + (Y_SHIFT_BLACK * TARGET_SIZE / 2.0)
            );
            Point outSample = new Point((
                    X_TOP_LEFT_R_TAG * TARGET_SIZE / 2.0) + (X_SHIFT_U * TARGET_SIZE / 2.0),
                    (Y_TOP_LEFT * TARGET_SIZE / 2.0) + (Y_SHIFT_U * TARGET_SIZE / 2.0)
            );

            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

            double blackVal = round(input.get((int) blackSample.y, (int) blackSample.x)[2] / 255.0 * 1000) / 1000.0;
            telemetry.addLine("Black value: " + blackVal);

            double whiteVal = round(input.get((int) whiteSample.y, (int) whiteSample.x)[2] / 255.0 * 1000) / 1000.0;
            telemetry.addLine("White value: " + whiteVal);

            double[] out = input.get((int) outSample.y, (int) outSample.x);
            telemetry.addLine(out[0] + ", " + out[1] + ", " + out[2]);

            double valBoost = 1.0 / (whiteVal - blackVal);

            // TODO remove for robot version
//            if (refresh) for (int[] row : slots) Arrays.fill(row, -1);

            for (int y = 0; y < points.length; y++) for (int x = 0; x < points[y].length; x++) {
                if (x == 0 && y % 2 == 0) continue;

                Point pointL = points[y][x][0];
                Point pointR = points[y][x][1];
                Point pointT = points[y][x][2];
                Point pointB = points[y][x][3];

                double[] colorL = input.get((int) pointL.y, (int) pointL.x);
                double[] colorR = input.get((int) pointR.y, (int) pointR.x);
                double[] colorT = input.get((int) pointT.y, (int) pointT.x);
                double[] colorB = input.get((int) pointB.y, (int) pointB.x);

                double avgHue = (colorL[0] + colorR[0] + colorT[0] + colorB[0]) / 2.0;
                double avgSat = (colorL[1] + colorR[1] + colorT[1] + colorB[1]) / 4.0 / 255.0;
                double avgVal = (colorL[2] + colorR[2] + colorT[2] + colorB[2]) / 4.0 / 255.0;

                double[] color = {
                        avgHue, // HUE IS MULTIPLIED BY 2 FOR RANGE [0, 360]
                        round(avgSat * 1000) / 1000.0,
                        clip(round((avgVal - blackVal) * valBoost * 1000) / 1000.0, 0, 1)
                };

                Pixel.Color c = hsvToColor(color);
                if (c != INVALID && c != backdrop.get(x, y).color) {
                    backdrop.add(new Pixel(x, y, c));
                }

                telemetry.addLine("(" + x + ", " + y + "), " + backdrop.get(x, y).color.name() + ": " + color[0] + ", " + color[1] + ", " + color[2]);
            }

            Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB);

            Imgproc.drawMarker(input, whiteSample, green, 2, 3);
            Imgproc.drawMarker(input, blackSample, green, 2, 3);
            Imgproc.drawMarker(input, outSample, green, 2, 3);

            if (graphic && background) {
                MatOfPoint background = new MatOfPoint(
                        CORNER_TL,
                        CORNER_TR,
                        CORNER_BR,
                        CORNER_BL
                );
                Imgproc.fillConvexPoly(
                        input,
                        background,
                        gray
                );
                background.release();
            }

            for (int y = 0; y < points.length; y++) for (int x = 0; x < points[y].length; x++) {
                if (x == 0 && y % 2 == 0) continue;
                Point center = new Point(
                        0.5 * (points[y][x][0].x + points[y][x][1].x),
                        0.5 * (points[y][x][0].y + points[y][x][1].y)
                );
                if (graphic) Imgproc.circle(input, center, 40, colorToScalar(backdrop.get(x, y).color), 8);
                Imgproc.putText(input, x + ", " + y, points[y][x][0], 2, 1, red);
            }

            if (!warp) {
                Mat inverseTransform = Imgproc.getPerspectiveTransform(dstTag, srcTag);
                Imgproc.warpPerspective(input, input, inverseTransform, input.size());
                inverseTransform.release();
            }
            srcTag.release();
            dstTag.release();

            Imgproc.line(input, tagTL, tagTR, yellow, 5);
            Imgproc.line(input, tagBL, tagBR, yellow, 5);
            Imgproc.line(input, tagTL, tagBL, yellow, 5);
            Imgproc.line(input, tagTR, tagBR, yellow, 5);
        }

        StringBuilder tagIds = new StringBuilder();
        for (AprilTagDetection tag : tags) tagIds.append(tag.id).append(" ");
        telemetry.addData("Detected tags", tagIds.toString());
        telemetry.update();

        return input;
    }

    private static Pixel.Color hsvToColor(double[] hsv) {
        return
                (hsv[0] == 0 && hsv[1] == 0 && hsv[2] == 0) ? INVALID :
                inRange(hsv, minPurple, maxPurple) ? PURPLE :
                inRange(hsv, minYellow, maxYellow) ? YELLOW :
                inRange(hsv, minGreen, maxGreen)   ? GREEN :
                inRange(hsv, minWhite, maxWhite)   ? WHITE :
                EMPTY
        ;
    }

    private static Scalar colorToScalar(Pixel.Color color) {
        return
                color == PURPLE ? lavender :
                color == YELLOW ? yellow :
                color == GREEN ? green :
                color == WHITE ? white :
                color == EMPTY ? black :
                gray
        ;
    }

    private static boolean inRange(double[] val, double[] lower, double[] upper) {
        return
                (val[0] >= lower[0] && val[0] <= upper[0]) &&
                (val[1] >= lower[1] && val[1] <= upper[1]) &&
                (val[2] >= lower[2] && val[2] <= upper[2])
        ;
    }

    private void generatePoints() {
        for (int y = 0; y < points.length; y++) {
            for (int x = 0; x < points[y].length; x++) {
                if (x == 0 && y % 2 == 0) continue;
                points[y][x][0] = pixelLeft(x, y);
                points[y][x][1] = pixelRight(x, y);
                points[y][x][2] = pixelTop(x, y);
                points[y][x][3] = pixelBottom(x, y);
            }
        }
    }

    private double getLeftX(int id) {
        return (X_TOP_LEFT_R_TAG * TARGET_SIZE / 2.0) - ((3 - (id - (id > 3 ? 3 : 0))) * (6 * (TARGET_SIZE / 2.0)));
    }

    private Point pixelLeft(int x, int y) {
        double width = 2.976 * (TARGET_SIZE / 2.0);
        return new Point(
                getLeftX(1) + (X_SHIFT_L_TAG_TO_L_PIXEL * TARGET_SIZE / 2.0) + (x * width) - (y % 2 == 0 ? 0.5 * width : 0),
                (Y_TOP_LEFT * TARGET_SIZE / 2.0) + (Y_SHIFT_TAG_TO_PIXEL * TARGET_SIZE / 2.0) - y * (2.625 * (TARGET_SIZE / 2.0))
        );
    }

    private Point pixelRight(int x, int y) {
        Point point = pixelLeft(x, y);
        point.x += (X_SHIFT_PIXEL_POINTS_R * TARGET_SIZE / 2.0);
        return point;
    }

    private Point pixelTop(int x, int y) {
        Point point = pixelLeft(x, y);
        point.x += (X_SHIFT_PIXEL_POINTS_R * TARGET_SIZE / 2.0) / 2.0;
        point.y += (Y_SHIFT_PIXEL_POINTS_T * TARGET_SIZE / 2.0);
        return point;
    }

    private Point pixelBottom(int x, int y) {
        Point point = pixelLeft(x, y);
        point.x += (X_SHIFT_PIXEL_POINTS_R * TARGET_SIZE / 2.0) / 2.0;
        point.y += (Y_SHIFT_PIXEL_POINTS_B * TARGET_SIZE / 2.0);
        return point;
    }
}
