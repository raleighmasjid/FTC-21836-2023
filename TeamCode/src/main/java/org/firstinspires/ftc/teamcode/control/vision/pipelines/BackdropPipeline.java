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

import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.black;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.blue;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.draw3dCubeMarker;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.drawAxisMarker;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.gray;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.green;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.lavender;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.poseFromTrapezoid;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.white;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.yellow;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.GREEN;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.INVALID;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.PURPLE;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.WHITE;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.YELLOW;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class BackdropPipeline extends OpenCvPipeline {

    public static final double SCREEN_HEIGHT = 1280, SCREEN_WIDTH = 720;

    public static final Point
            CORNER_TL = new Point(0, 0),
            CORNER_TR = new Point(SCREEN_WIDTH, 0),
            CORNER_BR = new Point(SCREEN_WIDTH, SCREEN_HEIGHT),
            CORNER_BL = new Point(0, SCREEN_HEIGHT);

    public boolean
            backdropVisible = false,
            isRed = true,
            graphic = true,
            background = false,
            showWarpPath = true;

    public double
            X_TOP_LEFT_R_TAG = 16.5,
            Y_TOP_LEFT = 32.42857142857142,
            TARGET_SIZE = 65,
            X_FIRST_PIXEL = 2.3285714285714287,
            Y_FIRST_PIXEL = 29.828571428571422,
            X_SHIFT_PIXEL_POINTS_L = -0.8714285714285714,
            X_SHIFT_PIXEL_POINTS_R = 0.8714285714285714,
            Y_SHIFT_PIXEL_POINTS_T = -1.1,
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

            minWhite =  {0, 0, 0.53},
            maxWhite =  {360, 0.242, 1};

    private final ArrayList<AprilTagDetection> tags = new ArrayList<>();

    private long nativeApriltagPtr;
    private final Mat grey = new Mat(), cameraMatrix;

    public final Backdrop backdrop = new Backdrop();
    private final Point[][] centerPoints = new Point[11][7];
    private final Point[][][] samplePoints = new Point[11][7][4];

    private final double
            fx = 1430,
            fy = 1430,
            cx = 480,
            cy = 620;

    public BackdropPipeline(Telemetry telemetry) {

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
        grey.release();

        // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
        // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
        if (showWarpPath) for (AprilTagDetection detection : detections) {
            AprilTagDetectionPipeline.Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagSize, tagSize);
            drawAxisMarker(input, tagSize / 2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
            draw3dCubeMarker(input, tagSize, tagSize, tagSize, 5, pose.rvec, pose.tvec, cameraMatrix);
        }

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

            if (showWarpPath) {
                Imgproc.line(input, tl, tr, blue, 3);
                Imgproc.line(input, bl, br, blue, 3);
                Imgproc.line(input, tl, bl, blue, 3);
                Imgproc.line(input, tr, br, blue, 3);
            }

            Mat transformMatrix = Imgproc.getPerspectiveTransform(srcTag, dstTag);
            Imgproc.warpPerspective(input, input, transformMatrix, input.size());
            transformMatrix.release();

            generateCenterPoints();
            generateSamplePoints();

            double size = 5;

            for (Point[] row : centerPoints) for (Point point : row) {
                if (point == null) continue;
                Imgproc.rectangle(
                        input,
                        new Point(point.x - size, point.y - size),
                        new Point(point.x + size, point.y + size),
                        blue,
                        2
                );
            }

//            for (Point[][] row : samplePoints) for (Point[] pair : row) for (Point point : pair) {
//                if (point == null) continue;
//                Imgproc.rectangle(
//                        input,
//                        new Point(point.x - size, point.y - size),
//                        new Point(point.x + size, point.y + size),
//                        blue,
//                        2
//                );
//            }
//
//            Point whiteSample = new Point(
//                    getLeftX(tags.get(0).id) + (X_SHIFT_WHITE * TARGET_SIZE / 2.0),
//                    (Y_TOP_LEFT * TARGET_SIZE / 2.0) + (Y_SHIFT_WHITE * TARGET_SIZE / 2.0)
//            );
//            Point blackSample = new Point(
//                    (X_TOP_LEFT_R_TAG * TARGET_SIZE / 2.0) + (X_SHIFT_BLACK * TARGET_SIZE / 2.0),
//                    (Y_TOP_LEFT * TARGET_SIZE / 2.0) + (Y_SHIFT_BLACK * TARGET_SIZE / 2.0)
//            );
//            Point outSample = new Point((
//                    X_TOP_LEFT_R_TAG * TARGET_SIZE / 2.0) + (X_SHIFT_U * TARGET_SIZE / 2.0),
//                    (Y_TOP_LEFT * TARGET_SIZE / 2.0) + (Y_SHIFT_U * TARGET_SIZE / 2.0)
//            );
//
//            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
//
//            double blackVal = round(input.get((int) blackSample.y, (int) blackSample.x)[2] / 255.0 * 1000) / 1000.0;
//            telemetry.addLine("Black value: " + blackVal);
//
//            double whiteVal = round(input.get((int) whiteSample.y, (int) whiteSample.x)[2] / 255.0 * 1000) / 1000.0;
//            telemetry.addLine("White value: " + whiteVal);
//
//            double[] out = input.get((int) outSample.y, (int) outSample.x);
//            telemetry.addLine(out[0] + ", " + out[1] + ", " + out[2]);
//
//            double valBoost = 1.0 / (whiteVal - blackVal);
//
//            for (int y = 0; y < samplePoints.length; y++) for (int x = 0; x < samplePoints[y].length; x++) {
//                if (x == 0 && y % 2 == 0) continue;
//
//                double hueSum = 0, satSum = 0, valSum = 0;
//
//                for (int i = 0; i < samplePoints[y][x].length; i++) {
//                    Point samplePoint = samplePoints[y][x][i];
//                    double[] sampleColor = input.get((int) samplePoint.y, (int) samplePoint.x);
//                    hueSum += sampleColor[0];
//                    satSum += sampleColor[1];
//                    valSum += sampleColor[2];
//                }
//
//                double avgHue = hueSum / ((double) samplePoints[y][x].length) * 2.0;
//                double avgSat = satSum / ((double) samplePoints[y][x].length) / 255.0;
//                double avgVal = valSum / ((double) samplePoints[y][x].length) / 255.0;
//
//                double[] color = {
//                        avgHue, // HUE IS MULTIPLIED BY 2 FOR RANGE [0, 360]
//                        round(avgSat * 1000) / 1000.0,
//                        clip(round((avgVal - blackVal) * valBoost * 1000) / 1000.0, 0, 1)
//                };
//
//                Pixel.Color c = hsvToColor(color);
//                if (c != INVALID && c != backdrop.get(x, y).color) {
//                    backdrop.add(new Pixel(x, y, c));
//                }
//
//                telemetry.addLine("(" + x + ", " + y + "), " + backdrop.get(x, y).color.name() + ": " + color[0] + ", " + color[1] + ", " + color[2]);
//            }
//
//            Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB);
//
//            Imgproc.drawMarker(input, whiteSample, green, 2, 3);
//            Imgproc.drawMarker(input, blackSample, green, 2, 3);
//            Imgproc.drawMarker(input, outSample, green, 2, 3);
//
//            if (graphic && background) {
//                MatOfPoint background = new MatOfPoint(
//                        CORNER_TL,
//                        CORNER_TR,
//                        CORNER_BR,
//                        CORNER_BL
//                );
//                Imgproc.fillConvexPoly(
//                        input,
//                        background,
//                        gray
//                );
//                background.release();
//            }
//
//            if (graphic) {
//                for (int y = 0; y < samplePoints.length; y++) for (int x = 0; x < samplePoints[y].length; x++) {
//                    if (x == 0 && y % 2 == 0) continue;
//                    Pixel pixel = backdrop.get(x, y);
//                    Imgproc.circle(input, centerPoints[y][x], 40, colorToScalar(pixel.color), 8);
//                    Imgproc.putText(input, x + ", " + y, samplePoints[y][x][0], 2, 1, red);
//                }
//
//                PlacementCalculator.getOptimalPlacements(backdrop);
//                for (int y = 0; y < samplePoints.length; y++) for (int x = 0; x < samplePoints[y].length; x++) {
//                    if (x == 0 && y % 2 == 0) continue;
//                    Pixel pixel = backdrop.get(x, y);
//                    if (pixel.inMosaic()) {
//                        Pixel[] mPixels = new Pixel[3];
//                        mPixels[0] = pixel;
//                        int i = 1;
//                        for (Pixel[] row : backdrop.slots) for (Pixel p : row) {
//                            if (pixel.mosaic == p.mosaic) {
//                                mPixels[i++] = p;
//                                if (i > 1) break;
//                            }
//                        }
//                        Point center = centerPoints[mPixels[0].y][mPixels[0].x];
//                        Point center2 = centerPoints[mPixels[1].y][mPixels[1].x];
//                        Point center3 = centerPoints[mPixels[2].y][mPixels[2].x];
//
//                        Imgproc.line(input, center, center2, blue, 5);
//                        Imgproc.line(input, center2, center3, blue, 5);
//                        Imgproc.line(input, center3, center, blue, 5);
//
//                    }
//                }
//            }
//            srcTag.release();
//            dstTag.release();
//
//            if (showWarpPath) {
//                Imgproc.line(input, tagTL, tagTR, yellow, 5);
//                Imgproc.line(input, tagBL, tagBR, yellow, 5);
//                Imgproc.line(input, tagTL, tagBL, yellow, 5);
//                Imgproc.line(input, tagTR, tagBR, yellow, 5);
//            }
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

    private void generateCenterPoints() {
        double width = 2.976 * (TARGET_SIZE / 2.0);
        double height = 2.625 * (TARGET_SIZE / 2.0);
        for (int y = 0; y < centerPoints.length; y++) for (int x = 0; x < centerPoints[y].length; x++) {
            if (x == 0 && y % 2 == 0) continue;
            centerPoints[y][x] = new Point(
                    (X_FIRST_PIXEL * TARGET_SIZE / 2.0) + (x * width) - (y % 2 == 0 ? 0.5 * width : 0),
                    (Y_FIRST_PIXEL * TARGET_SIZE / 2.0) - (y * height)
            );
        }
    }

    private void generateSamplePoints() {
        for (int y = 0; y < samplePoints.length; y++) for (int x = 0; x < samplePoints[y].length; x++) {
            if (x == 0 && y % 2 == 0) continue;
            samplePoints[y][x][0] = pixelLeft(x, y);
            samplePoints[y][x][1] = pixelRight(x, y);
            samplePoints[y][x][2] = pixelTop(x, y);
            samplePoints[y][x][3] = pixelBottom(x, y);
        }
    }

    private double getLeftX(int id) {
        return (X_TOP_LEFT_R_TAG * TARGET_SIZE / 2.0) - ((3 - (id - (id > 3 ? 3 : 0))) * (6 * (TARGET_SIZE / 2.0)));
    }

    private Point pixelLeft(int x, int y) {
        Point point = centerPoints[y][x].clone();
        point.x += (X_SHIFT_PIXEL_POINTS_L * TARGET_SIZE / 2.0);
        return point;
    }

    private Point pixelRight(int x, int y) {
        Point point = centerPoints[y][x].clone();
        point.x += (X_SHIFT_PIXEL_POINTS_R * TARGET_SIZE / 2.0);
        return point;
    }

    private Point pixelTop(int x, int y) {
        Point point = centerPoints[y][x].clone();
        point.y += (Y_SHIFT_PIXEL_POINTS_T * TARGET_SIZE / 2.0);
        return point;
    }

    private Point pixelBottom(int x, int y) {
        Point point = centerPoints[y][x].clone();
        point.y += (Y_SHIFT_PIXEL_POINTS_B * TARGET_SIZE / 2.0);
        return point;
    }
}
