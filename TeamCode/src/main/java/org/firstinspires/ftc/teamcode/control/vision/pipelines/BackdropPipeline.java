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
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.GREEN;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.INVALID;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.PURPLE;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.WHITE;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.YELLOW;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.PlacementCalculator.iterationXs;
import static org.opencv.imgproc.Imgproc.INTER_AREA;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.round;
import static java.lang.Math.sqrt;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.PlacementCalculator;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class BackdropPipeline extends OpenCvPipeline {

    public boolean
            backdropVisible = false,
            isRed = true,
            showGraphics = true,
            showSamples = false,
            showBackground = false,
            showCircleDet = false,
            logicEnhancements = true;

    public int futureSteps = 1;

    private static final double
            SCALING_FACTOR = 1 / 8.0,
            SCREEN_HEIGHT = 1280 * SCALING_FACTOR,
            SCREEN_WIDTH = 720 * SCALING_FACTOR,
            X_TOP_LEFT_R_TAG = 536.25 * SCALING_FACTOR,
            Y_TOP_LEFT = 1053.9285714285713 * SCALING_FACTOR,
            TARGET_SIZE = 65 * SCALING_FACTOR,
            X_FIRST_PIXEL = 128.9125 * SCALING_FACTOR,
            Y_FIRST_PIXEL = 966.875 * SCALING_FACTOR,
            X_SHIFT_PIXEL_POINTS_L = -28.321428571428573 * SCALING_FACTOR,
            X_SHIFT_PIXEL_POINTS_R = 28.321428571428573 * SCALING_FACTOR,
            Y_SHIFT_PIXEL_POINTS_T = -35.75 * SCALING_FACTOR,
            Y_SHIFT_PIXEL_POINTS_B = 24.142857142857142 * SCALING_FACTOR,
            HEX_RADIUS = 80 * SCALING_FACTOR,
            HEX_SIDE_LENGTH = HEX_RADIUS / sqrt(3),
            PLACEMENT_MARKER_RADIUS = 30 * SCALING_FACTOR,
            X_DIST_GRID = 97.825 * SCALING_FACTOR,
            Y_DIST_GRID = -85.0 * SCALING_FACTOR,
            CIRCLE_DET_BLUR = 16 * SCALING_FACTOR,
            fx = 1430,
            fy = 1430,
            cx = 480,
            cy = 620;

    private final Size scalar = new Size(SCALING_FACTOR, SCALING_FACTOR);

    private Point scale(Point p) {
        return new Point((p.x * SCALING_FACTOR), (p.y * SCALING_FACTOR));
    }

    private final Point
            CORNER_TL = new Point(0, 0),
            CORNER_TR = new Point(SCREEN_WIDTH, 0),
            CORNER_BR = new Point(SCREEN_WIDTH, SCREEN_HEIGHT),
            CORNER_BL = new Point(0, SCREEN_HEIGHT);

    private Point
            tagTR = new Point(),
            tagBL = new Point(),
            tagTL = new Point(),
            tagBR = new Point();

    private static final double[]
            minPurple = {140, .15, .3},
            maxPurple = {320, 1, 1},

            minYellow = {20, .51, .5},
            maxYellow = {50, 1, 1},

            minGreen =  {70, .4, .215},
            maxGreen =  {110, 1, 1},

            minWhite =  {0, 0, 0.65},
            maxWhite =  {360, 0.242, 1};

    private final ArrayList<AprilTagDetection> tags = new ArrayList<>();

    private long nativeApriltagPtr;
    private final Mat grey = new Mat(), cameraMatrix, warpedGray = new Mat(), circles = new Mat();

    public final Backdrop backdrop;
    private final PlacementCalculator calculator = new PlacementCalculator();
    private final Telemetry telemetry;

    private final Point[][] centerPoints = new Point[11][7];
    private final double[][][] averageHSVs = new double[11][7][3];

    private final Point[][][] samplePoints = new Point[11][7][4];
    private final double[][][][] sampleHSVs = new double[11][7][4][3];

    private final Point[][][] hexCorners = new Point[11][7][6];

    private final Size CIRCLE_DET_SIZE = new Size(CIRCLE_DET_BLUR, CIRCLE_DET_BLUR);

    private final Backdrop recordOfHighestPixel = new Backdrop();

    public BackdropPipeline(Telemetry telemetry, Backdrop backdrop) {

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
        this.backdrop = backdrop;

        generateCenterPoints();
        generateSamplePoints();
        generateHexCorners();
    }

    public BackdropPipeline(Telemetry telemetry) {
        this(telemetry, new Backdrop());
    }

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
        if (showSamples) for (AprilTagDetection detection : detections) {
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
            if (id != left && id != middle && id != right) continue;
            tags.add(detection);
        }

        backdropVisible = !tags.isEmpty();

        if (backdropVisible) {

            if (SCALING_FACTOR != 1) Imgproc.resize(input, input, scalar, SCALING_FACTOR, SCALING_FACTOR, INTER_AREA);

            int minInd = 0, maxInd = 0;
            for (int i = 0; i < tags.size(); i++) {
                if (tags.get(i).id < tags.get(minInd).id) minInd = i;
                if (tags.get(i).id > tags.get(maxInd).id) maxInd = i;
                if (tags.size() >= 3 || tags.get(i).id % 3 != 2) continue;
                minInd = maxInd = i;
                break;
            }

            warpImageToStraightenBackdrop(input, minInd, maxInd);

            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

            warpToFitGrid(input);
            saveBackdropColors(input);

            Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB);

            if (showSamples) drawSamplingMarkers(input);

            if (showGraphics) drawGraphics(input);
        }

        StringBuilder tagIds = new StringBuilder();
        for (AprilTagDetection tag : tags) tagIds.append(tag.id).append(" ");
        telemetry.addData("Detected tags", tagIds.toString());
        telemetry.update();

        if (showCircleDet) return warpedGray;
        warpedGray.release();
        return input;
    }

    private void warpImageToStraightenBackdrop(Mat input, int minInd, int maxInd) {
        Point bl = scale(tags.get(minInd).corners[0]);
        Point tl = scale(tags.get(minInd).corners[3]);
        Point br = scale(tags.get(maxInd).corners[1]);
        Point tr = scale(tags.get(maxInd).corners[2]);

        MatOfPoint2f srcTag = new MatOfPoint2f(
                bl,
                br,
                tr,
                tl
        );

        double leftX = getLeftX(tags.get(minInd).id);
        double rightX = getLeftX(tags.get(maxInd).id) + TARGET_SIZE;

        tagTR = new Point(rightX, (Y_TOP_LEFT));
        tagBL = new Point(leftX, (Y_TOP_LEFT) + TARGET_SIZE);
        tagTL = new Point(leftX, (Y_TOP_LEFT));
        tagBR = new Point(rightX, (Y_TOP_LEFT) + TARGET_SIZE);

        MatOfPoint2f dstTag = new MatOfPoint2f(
                tagBL,
                tagBR,
                tagTR,
                tagTL
        );

        if (showSamples) {
            Imgproc.line(input, tl, tr, blue, 1);
            Imgproc.line(input, bl, br, blue, 1);
            Imgproc.line(input, tl, bl, blue, 1);
            Imgproc.line(input, tr, br, blue, 1);
        }

        Mat transformMatrix = Imgproc.getPerspectiveTransform(srcTag, dstTag);
        Imgproc.warpPerspective(input, input, transformMatrix, input.size());
        srcTag.release();
        dstTag.release();
        transformMatrix.release();

        Imgproc.cvtColor(input, warpedGray, Imgproc.COLOR_RGBA2GRAY);
    }

    private void drawGraphics(Mat input) {
        if (showBackground) {
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

        for (int y = 0; y < centerPoints.length; y++) for (int x = 0; x < centerPoints[y].length; x++) {
            if (x == 0 && y % 2 == 0) continue;
            drawPixelIcon(input, y, x);
        }

        ArrayList<Pixel> optimalPlacements = calculator.getOptimalPlacements(backdrop);
        for (int y = 0; y < centerPoints.length; y++) for (int x = 0; x < centerPoints[y].length; x++) {
            if (x == 0 && y % 2 == 0) continue;
            Pixel pixel = backdrop.get(x, y);
            if (pixel.inMosaic()) drawMosaic(input, pixel);
        }

        if (optimalPlacements.isEmpty()) return;

        Pixel p = optimalPlacements.get(0);
        Imgproc.circle(input, centerPoints[p.y][p.x], (int) PLACEMENT_MARKER_RADIUS, colorToScalar(p.color), (int) (8 * SCALING_FACTOR));

        for (Pixel a : optimalPlacements) telemetry.addLine(a.toString());

        if (futureSteps > 1) for (int i = 1; i < futureSteps; i++) {
            optimalPlacements = calculator.getOptimalPlacements(backdrop.add(p));

            if (optimalPlacements.isEmpty()) return;

            p = optimalPlacements.get(0);
            Imgproc.circle(input, centerPoints[p.y][p.x], (int) PLACEMENT_MARKER_RADIUS, colorToScalar(p.color), (int) (8 * SCALING_FACTOR));
        }
    }

    private void drawSamplingMarkers(Mat input) {
        for (Point[] centerPoint : centerPoints) for (Point point : centerPoint) {
            if (point == null) continue;
            Imgproc.drawMarker(input, point, blue, 2, 1);
        }

        for (Point[][] row : samplePoints) for (Point[] group : row) for (Point point : group) {
            if (point == null) continue;
            Imgproc.drawMarker(input, point, blue, 2, 1);
        }

        Imgproc.line(input, tagTL, tagTR, yellow, 1);
        Imgproc.line(input, tagBL, tagBR, yellow, 1);
        Imgproc.line(input, tagTL, tagBL, yellow, 1);
        Imgproc.line(input, tagTR, tagBR, yellow, 1);
    }

    private void saveBackdropColors(Mat input) {

        backdrop.clear();

        for (int y = 0; y < centerPoints.length; y++) for (int x = 0; x < centerPoints[y].length; x++) {
            if (x == 0 && y % 2 == 0) continue;

            double[] color = getColorOfPixel(input, y, x);

            Pixel.Color c = hsvToColor(color);
            telemetry.addLine("(" + x + ", " + y + "), " + c.name() + ": " + color[0] + ", " + color[1] + ", " + color[2]);
            if (c == INVALID || c == backdrop.get(x, y).color) continue;
            Pixel pixel = new Pixel(x, y, c);
            if (logicEnhancements && c != EMPTY && !backdrop.isSupported(pixel)) continue;
            backdrop.add(pixel);
        }
    }

    private void drawMosaic(Mat input, Pixel pixel) {
        Pixel[] mPixels = new Pixel[3];
        int i = 0;
        pixels:
        for (Pixel[] row : backdrop.slots) for (Pixel p : row) {
            if (pixel.mosaic == p.mosaic) {
                if (i > 2) break pixels;
                else mPixels[i++] = p;
            }
        }
        Point center1 = centerPoints[mPixels[0].y][mPixels[0].x];
        Point center2 = centerPoints[mPixels[1].y][mPixels[1].x];
        Point center3 = centerPoints[mPixels[2].y][mPixels[2].x];

        Imgproc.line(input, center1, center2, blue, max(1, (int) (5 * SCALING_FACTOR)));
        Imgproc.line(input, center2, center3, blue, max(1, (int) (5 * SCALING_FACTOR)));
        Imgproc.line(input, center3, center1, blue, max(1, (int) (5 * SCALING_FACTOR)));
    }

    private void warpToFitGrid(Mat input) {
        recordOfHighestPixel.clear();

        Point[] target = {
                new Point(), // tl =
                new Point(), // br =
                new Point()  // bl =
        };

        for (int y = 0; y < centerPoints.length; y++) for (int x : iterationXs(y)) {
            if (x == 0 && y % 2 == 0) continue;

            Pixel.Color color = hsvToColor(getColorOfPixel(input, y, x));
            switch (color) {
                case EMPTY: case INVALID: continue;
            }

            Pixel pixel = new Pixel(x, y, color);
            if (logicEnhancements && !recordOfHighestPixel.isSupported(pixel)) continue;

            target[0].x = x;
            target[0].y = y;
            if (logicEnhancements) recordOfHighestPixel.add(pixel);
        }

        a:
        for (int y = 0; y < centerPoints.length; y++) for (int x = 0; x < centerPoints[y].length; x++) {
            if (x == 0 && y % 2 == 0) continue;

            Pixel.Color color = hsvToColor(getColorOfPixel(input, y, x));
            switch (color) {
                case EMPTY: case INVALID: continue;
            }

            target[2].x = x;
            target[2].y = y;
            break a;
        }

        b:
        for (int y = 0; y < centerPoints.length; y++) for (int x = centerPoints[y].length - 1; x >= 0; x--) {
            if (x == 0 && y % 2 == 0) continue;

            Pixel.Color color = hsvToColor(getColorOfPixel(input, y, x));
            switch (color) {
                case EMPTY: case INVALID: continue;
            }

            if (x == target[2].x && y == target[2].y) continue;

            target[1].x = x;
            target[1].y = y;
            break b;
        }

        telemetry.addLine("(" + target[0].x + ", " + target[0].y + ")");
        telemetry.addLine("(" + target[1].x + ", " + target[1].y + ")");
        telemetry.addLine("(" + target[2].x + ", " + target[2].y + ")");

        boolean valid = !(
                target[1].equals(target[0]) ||
                target[1].equals(target[2]) ||
                target[0].equals(target[2])
        );
        telemetry.addLine("Valid affine warp: " + valid);

        if (!valid) return;

        Point[] source = {
                findCenter(target[0]),
                coordToPoint(target[1]),
                coordToPoint(target[2])
        };

        MatOfPoint2f src = new MatOfPoint2f(
                source[0],
                source[1],
                source[2]
        );

        Point[] targetPixels = {
                coordToPoint(target[0]),
                coordToPoint(target[1]),
                coordToPoint(target[2])
        };

        MatOfPoint2f dst = new MatOfPoint2f(
                targetPixels[0],
                targetPixels[1],
                targetPixels[2]
        );

        boolean warp = true;

        if (!warp) {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB);

            Imgproc.line(input, source[0], source[1], blue, 3);
            Imgproc.line(input, source[1], source[2], blue, 3);
            Imgproc.line(input, source[2], source[0], blue, 3);
            Imgproc.line(input, targetPixels[0], targetPixels[1], blue, 3);
            Imgproc.line(input, targetPixels[1], targetPixels[2], blue, 3);
            Imgproc.line(input, targetPixels[2], targetPixels[0], blue, 3);
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
            return;
        }

        Mat transformMatrix = Imgproc.getAffineTransform(src, dst);
        src.release();
        dst.release();
        Imgproc.warpAffine(input, input, transformMatrix, input.size());
        transformMatrix.release();
    }

    private Point findCenter(Point p) {
        Point estimate = coordToPoint(p);
        int boxRadius = (int) (45 * SCALING_FACTOR);
        Point topLeft = new Point(
                clipX(estimate.x - boxRadius),
                clipY(estimate.y - boxRadius)
        );
        Mat region = warpedGray.submat(new Rect(
                topLeft,
                new Point(
                        clipX(estimate.x + boxRadius),
                        clipY(estimate.y + boxRadius)
                )
        ));

        int blockSize = (int) (61 * SCALING_FACTOR);
        if (blockSize % 2 == 0) blockSize++;
        Imgproc.adaptiveThreshold(region, region, 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY, blockSize, -1);
        Imgproc.blur(region, region, CIRCLE_DET_SIZE);
        
        circles.release();
        Imgproc.HoughCircles(region, circles, Imgproc.HOUGH_GRADIENT, 1.225, 500 * SCALING_FACTOR, 1, .5, 2, -1);
        region.release();

        if (circles.size().width > 0) {

            estimate = new Point(
                    topLeft.x + circles.get(0, 0)[0],
                    topLeft.y + circles.get(0, 0)[1]
            );
        }
        circles.release();
        Imgproc.drawMarker(warpedGray, estimate, red, 2, (int) (8 * SCALING_FACTOR));
        return estimate;
    }

    private Point coordToPoint(Point p) {
        return centerPoints[(int) p.y][(int) p.x];
    }

    private void drawPixelIcon(Mat input, int y, int x) {
        Scalar color = colorToScalar(backdrop.get(x, y).color);

        for (int i = 0; i < hexCorners[y][x].length; i++) {
            int i2 = loopClip(i + 1, hexCorners[y][x].length);
            Imgproc.line(
                    input,
                    hexCorners[y][x][i],
                    hexCorners[y][x][i2],
                    color,
                    (int) (8 * SCALING_FACTOR)
            );
        }

        Imgproc.putText(input, x + ", " + y, hexCorners[y][x][5], 2, SCALING_FACTOR, red);
    }

    public static int loopClip(int a, int b) {
        return (int) loopClip(a,(double) b);
    }

    public static double loopClip(double a, double b) {
        return (a % b + b) % b;
    }

    private double[] getColorOfPixel(Mat input, int y, int x) {
        double hueSum = 0, satSum = 0, valSum = 0;

        double sampleCount = samplePoints[y][x].length;

        for (int i = 0; i < samplePoints[y][x].length; i++) {
            Point samplePoint = samplePoints[y][x][i];
            sampleHSVs[y][x][i] = input.get((int) samplePoint.y, (int) samplePoint.x);

            if (voidHSV(sampleHSVs[y][x][i])) {
                sampleCount--;
                continue;
            }

            hueSum += sampleHSVs[y][x][i][0];
            satSum += sampleHSVs[y][x][i][1];
            valSum += sampleHSVs[y][x][i][2];
        }

        if (sampleCount == 0) {

            averageHSVs[y][x][0] = 0;
            averageHSVs[y][x][1] = 0;
            averageHSVs[y][x][2] = 0;

        } else {

            double avgHue = hueSum / sampleCount * 2.0; // HUE IS MULTIPLIED BY 2 FOR RANGE [0, 360]
            double avgSat = satSum / sampleCount / 255.0;
            double avgVal = valSum / sampleCount / 255.0;

            averageHSVs[y][x][0] = avgHue;
            averageHSVs[y][x][1] = round(avgSat * 1000) / 1000.0;
            averageHSVs[y][x][2] = round(avgVal * 1000) / 1000.0;
        }

        return averageHSVs[y][x];
    }

    private static Pixel.Color hsvToColor(double[] hsv) {
        return
                voidHSV(hsv) ? INVALID :
                inRange(hsv, minPurple, maxPurple) ? PURPLE :
                inRange(hsv, minYellow, maxYellow) ? YELLOW :
                inRange(hsv, minGreen, maxGreen)   ? GREEN :
                inRange(hsv, minWhite, maxWhite)   ? WHITE :
                EMPTY
        ;
    }

    private static boolean voidHSV(double[] hsv) {
        return hsv[0] == 0 && hsv[1] == 0 && hsv[2] == 0;
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
        for (int y = 0; y < centerPoints.length; y++) for (int x = 0; x < centerPoints[y].length; x++) {
            boolean evenRow = y % 2 == 0;
            if (x == 0 && evenRow) continue;

            centerPoints[y][x] = new Point(
                    clipX(X_FIRST_PIXEL + ((x - 1) * X_DIST_GRID) + (!evenRow ? 0.5 * X_DIST_GRID : 0)),
                    clipY(Y_FIRST_PIXEL + (y * Y_DIST_GRID))
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

    private void generateHexCorners() {
        for (int y = 0; y < hexCorners.length; y++) for (int x = 0; x < hexCorners[y].length; x++) {
            if (x == 0 && y % 2 == 0) continue;

            hexCorners[y][x] = new Point[]{
                    centerPoints[y][x].clone(),
                    centerPoints[y][x].clone(),
                    centerPoints[y][x].clone(),
                    centerPoints[y][x].clone(),
                    centerPoints[y][x].clone(),
                    centerPoints[y][x].clone(),
            };

            hexCorners[y][x][1].x += 0.5 * HEX_RADIUS;
            hexCorners[y][x][1].y += 0.5 * HEX_SIDE_LENGTH;

            hexCorners[y][x][2].x += 0.5 * HEX_RADIUS;
            hexCorners[y][x][2].y -= 0.5 * HEX_SIDE_LENGTH;

            hexCorners[y][x][4].x -= 0.5 * HEX_RADIUS;
            hexCorners[y][x][4].y -= 0.5 * HEX_SIDE_LENGTH;

            hexCorners[y][x][5].x -= 0.5 * HEX_RADIUS;
            hexCorners[y][x][5].y += 0.5 * HEX_SIDE_LENGTH;

            hexCorners[y][x][0].y += HEX_SIDE_LENGTH;
            hexCorners[y][x][3].y -= HEX_SIDE_LENGTH;

        }
    }

    private double getLeftX(int id) {
        return (X_TOP_LEFT_R_TAG) - ((3 - (id - (id > 3 ? 3 : 0))) * (6 * (TARGET_SIZE / 2.0)));
    }

    private Point pixelLeft(int x, int y) {
        Point point = centerPoints[y][x].clone();
        point.x = clipX(point.x + (X_SHIFT_PIXEL_POINTS_L));
        return point;
    }

    private Point pixelRight(int x, int y) {
        Point point = centerPoints[y][x].clone();
        point.x = clipX(point.x + (X_SHIFT_PIXEL_POINTS_R));
        return point;
    }

    private Point pixelTop(int x, int y) {
        Point point = centerPoints[y][x].clone();
        point.y = clipY(point.y + (Y_SHIFT_PIXEL_POINTS_T));
        return point;
    }

    private Point pixelBottom(int x, int y) {
        Point point = centerPoints[y][x].clone();
        point.y = clipY(point.y + (Y_SHIFT_PIXEL_POINTS_B));
        return point;
    }

    private double clipY(double y) {
        return min(max(y, 0), SCREEN_HEIGHT);
    }

    private double clipX(double x) {
        return min(max(x, 0), SCREEN_WIDTH);
    }
}
