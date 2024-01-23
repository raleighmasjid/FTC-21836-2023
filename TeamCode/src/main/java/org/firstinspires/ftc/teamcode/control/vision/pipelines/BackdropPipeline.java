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
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.red;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.white;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.AprilTagDetectionPipeline.yellow;
import static java.lang.Math.min;
import static java.lang.Math.round;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import java.util.Arrays;

public class BackdropPipeline extends OpenCvPipeline {

    public static final double SCREEN_HEIGHT = 1280, SCREEN_WIDTH = 960;

    public static final Point
            CORNER_TL = new Point(0, 0),
            CORNER_TR = new Point(SCREEN_WIDTH, 0),
            CORNER_BR = new Point(SCREEN_WIDTH, SCREEN_HEIGHT),
            CORNER_BL = new Point(0, SCREEN_HEIGHT);

    public boolean
            warp = true,
            backdropVisible = false,
            isRed = true,
            editPoints = false,
            graphic = true,
            background = false;

    public double
            X_TOP_LEFT_R_TAG = 660,
            Y_TOP_LEFT = 1100,
            TARGET_SIZE = 70,
            X_SHIFT_L_TAG_TO_L_PIXEL = -100,
            Y_SHIFT_TAG_TO_PIXEL = -90,
            X_SHIFT_PIXEL_POINTS_R = 66,
            Y_SHIFT_PIXEL_POINTS_T = -40,
            Y_SHIFT_PIXEL_POINTS_B = 26,
            X_SHIFT_WHITE = 3,
            Y_SHIFT_WHITE = 85,
            X_SHIFT_BLACK = 200,
            Y_SHIFT_BLACK = 0;

    private static final double[]
            minPurple = {140, .15, .3},
            maxPurple = {320, 1, 1},

            minYellow = {20, .51, .65},
            maxYellow = {50, 1, 1},

            minGreen =  {85, .4, .25},
            maxGreen =  {110, 1, 1},

            minWhite =  {0, 0, 0.7},
            maxWhite =  {360, 0.2, 1},

            minBlack =  {0, 0, 0},
            maxBlack =  {360, 1, 0.3};

    private final ArrayList<AprilTagDetection> tags = new ArrayList<>();

    private long nativeApriltagPtr;
    private final Mat grey = new Mat(), cameraMatrix;

    public final int[][] slots = new int[11][7];
    private final Point[][][] points = new Point[11][7][4];

    private final double
            fx = 1430,
            fy = 1430,
            cx = 480,
            cy = 620;

    public BackdropPipeline(Telemetry telemetry) {

        for (int[] row : slots) Arrays.fill(row, 4);

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
                    tagTR = new Point(rightX, Y_TOP_LEFT),
                    tagBL = new Point(leftX, Y_TOP_LEFT + TARGET_SIZE),
                    tagTL = new Point(leftX, Y_TOP_LEFT),
                    tagBR = new Point(rightX, Y_TOP_LEFT + TARGET_SIZE);

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

            if (warp) {
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

                Point whiteSample = new Point(X_TOP_LEFT_R_TAG + X_SHIFT_WHITE, Y_TOP_LEFT + Y_SHIFT_WHITE);
                Point blackSample = new Point(X_TOP_LEFT_R_TAG + X_SHIFT_BLACK, Y_TOP_LEFT + Y_SHIFT_BLACK);

                Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

                double blackVal = round(input.get((int) blackSample.y, (int) blackSample.x)[2] / 255.0 * 1000) / 1000.0;
                telemetry.addLine("Black value: " + blackVal);

                double whiteVal = round(input.get((int) whiteSample.y, (int) whiteSample.x)[2] / 255.0 * 1000) / 1000.0;
                telemetry.addLine("White value: " + whiteVal);

                double valBoost = 1.0 / (whiteVal - blackVal);

                // TODO remove for robot version
                for (int[] row : slots) Arrays.fill(row, 4);

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
                            min(round((avgVal - blackVal) * valBoost * 1000) / 1000.0, 1.0)
                    };

                    int colorInt = hsvToColorInt(color);
                    if (colorInt >- 1) slots[y][x] = colorInt;

                    telemetry.addLine("(" + x + ", " + y + "), " + colorIntToString(slots[y][x]) + ": " + color[0] + ", " + color[1] + ", " + color[2]);
                }

                Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB);

                Imgproc.drawMarker(input, whiteSample, green, 2, 3);
                Imgproc.drawMarker(input, blackSample, green, 2, 3);

                if (graphic && background) Imgproc.fillConvexPoly(
                        input,
                        new MatOfPoint(
                                CORNER_TL,
                                CORNER_TR,
                                CORNER_BR,
                                CORNER_BL
                        ),
                        gray
                );

                for (int y = 0; y < points.length; y++) for (int x = 0; x < points[y].length; x++) {
                    if (x == 0 && y % 2 == 0) continue;
                    Point center = new Point(
                            0.5 * (points[y][x][0].x + points[y][x][1].x),
                            0.5 * (points[y][x][0].y + points[y][x][1].y)
                    );
                    if (graphic) Imgproc.circle(input, center, 40, colorIntToScalar(slots[y][x]), 8);
                    Imgproc.putText(input, x + ", " + y, points[y][x][0], 2, 1, red);
                }

            }

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

    private static int hsvToColorInt(double[] hsv) {
        return
                inRange(hsv, minPurple, maxPurple) ? 0 :
                inRange(hsv, minYellow, maxYellow) ? 1 :
                inRange(hsv, minGreen, maxGreen)   ? 2 :
                inRange(hsv, minWhite, maxWhite)   ? 3 :
                inRange(hsv, minBlack, maxBlack)   ? 4 :
                -1
        ;
    }

    private static String colorIntToString(int colorInt) {
        switch (colorInt) {
            case 0: return "PURPLE";
            case 1: return "YELLOW";
            case 2: return "GREEN";
            case 3: return "WHITE";
            case 4: return "EMPTY";
            default: return "UNKNOWN";
        }
    }

    private static Scalar colorIntToScalar(int colorInt) {
        switch (colorInt) {
            case 0: return lavender;
            case 1: return yellow;
            case 2: return green;
            case 3: return white;
            case 4: return black;
            default: return gray;
        }
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
        return X_TOP_LEFT_R_TAG - ((3 - (id - (id > 3 ? 3 : 0))) * (6 * (TARGET_SIZE / 2.0)));
    }

    private Point pixelLeft(int x, int y) {
        double width = 2.976 * (TARGET_SIZE / 2.0);
        return new Point(
                getLeftX(1) + X_SHIFT_L_TAG_TO_L_PIXEL + (x * width) - (y % 2 == 0 ? 0.5 * width : 0),
                Y_TOP_LEFT + Y_SHIFT_TAG_TO_PIXEL - y * (2.48 * (TARGET_SIZE / 2.0))
        );
    }

    private Point pixelRight(int x, int y) {
        Point point = pixelLeft(x, y);
        point.x += X_SHIFT_PIXEL_POINTS_R;
        return point;
    }

    private Point pixelTop(int x, int y) {
        Point point = pixelLeft(x, y);
        point.x += X_SHIFT_PIXEL_POINTS_R / 2.0;
        point.y += Y_SHIFT_PIXEL_POINTS_T;
        return point;
    }

    private Point pixelBottom(int x, int y) {
        Point point = pixelLeft(x, y);
        point.x += X_SHIFT_PIXEL_POINTS_R / 2.0;
        point.y += Y_SHIFT_PIXEL_POINTS_B;
        return point;
    }
}
