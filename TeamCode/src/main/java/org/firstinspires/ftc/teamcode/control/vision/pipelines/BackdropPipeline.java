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
import static org.opencv.imgproc.Imgproc.INTER_AREA;
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
            showCircleDet = false;

    private static final double
            SCALING_FACTOR = 1 / 7.0,
            SCREEN_HEIGHT = 1280 * SCALING_FACTOR,
            SCREEN_WIDTH = 720 * SCALING_FACTOR,
            X_TOP_LEFT_R_TAG = 536.25 * SCALING_FACTOR,
            Y_TOP_LEFT = 1053.9285714285713 * SCALING_FACTOR,
            TARGET_SIZE = 65 * SCALING_FACTOR,
            X_FIRST_PIXEL = 80 * SCALING_FACTOR,
            Y_FIRST_PIXEL = 966.875 * SCALING_FACTOR,
            X_SHIFT_PIXEL_POINTS_L = -28.321428571428573 * SCALING_FACTOR,
            X_SHIFT_PIXEL_POINTS_R = 28.321428571428573 * SCALING_FACTOR,
            Y_SHIFT_PIXEL_POINTS_T = -35.75 * SCALING_FACTOR,
            Y_SHIFT_PIXEL_POINTS_B = 24.142857142857142 * SCALING_FACTOR,
            fx = 1430,
            fy = 1430,
            cx = 480,
            cy = 620;

    private static final Size scalar = new Size(SCALING_FACTOR, SCALING_FACTOR);

    private static Point scale(Point p) {
        return new Point((p.x * SCALING_FACTOR), (p.y * SCALING_FACTOR));
    }

    private static final Point
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

            minWhite =  {0, 0, 0.7},
            maxWhite =  {360, 0.242, 1};

    private final ArrayList<AprilTagDetection> tags = new ArrayList<>();

    private long nativeApriltagPtr;
    private final Mat grey = new Mat(), cameraMatrix, warpedGray = new Mat();

    public final Backdrop backdrop;
    private final Telemetry telemetry;

    private final Point[][] centerPoints = new Point[11][7];
    private final Point[][][] samplePoints = new Point[11][7][4];

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

            Imgproc.resize(input, input, scalar, SCALING_FACTOR, SCALING_FACTOR, INTER_AREA);

            int minInd = 0, maxInd = 0;
            for (int i = 0; i < tags.size(); i++) {
                if (tags.get(i).id < tags.get(minInd).id) minInd = i;
                if (tags.get(i).id > tags.get(maxInd).id) maxInd = i;
                if (tags.size() >= 3 || tags.get(i).id % 3 != 2) continue;
                minInd = maxInd = i;
                break;
            }

            warpImageToStraightenBackdrop(input, minInd, maxInd);

            if (showSamples) {
                Imgproc.line(input, tagTL, tagTR, yellow, 1);
                Imgproc.line(input, tagBL, tagBR, yellow, 1);
                Imgproc.line(input, tagTL, tagBL, yellow, 1);
                Imgproc.line(input, tagTR, tagBR, yellow, 1);
            }

            generateCenterPoints();
            generateSamplePoints();

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

        return showCircleDet ? warpedGray : input;
    }

    private static double[] getHSV(Mat input, Point point) {
        return input.get((int) point.y, (int) point.x);
    }

    private static double getValue(Mat input, Point point) {
        return round(getHSV(input, point)[2] / 255.0 * 1000) / 1000.0;
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

        PlacementCalculator.getOptimalPlacements(backdrop);
        for (int y = 0; y < centerPoints.length; y++) for (int x = 0; x < centerPoints[y].length; x++) {
            if (x == 0 && y % 2 == 0) continue;
            Pixel pixel = backdrop.get(x, y);
            if (pixel.inMosaic()) drawMosaic(input, pixel);
        }
    }

    private void drawSamplingMarkers(Mat input) {
        for (Point[] centerPoint : centerPoints) for (Point point : centerPoint) {
            drawBlueSquare(input, point);
        }

        for (Point[][] row : samplePoints) for (Point[] pair : row) {
            for (Point point : pair) drawBlueSquare(input, point);
        }

        Imgproc.line(input, tagTL, tagTR, yellow, 1);
        Imgproc.line(input, tagBL, tagBR, yellow, 1);
        Imgproc.line(input, tagTL, tagBL, yellow, 1);
        Imgproc.line(input, tagTR, tagBR, yellow, 1);
    }

    private void saveBackdropColors(Mat input) {

        for (int y = 0; y < centerPoints.length; y++) for (int x = 0; x < centerPoints[y].length; x++) {
            if (x == 0 && y % 2 == 0) continue;

            double[] color = getColorOfPixel(input, y, x);

            Pixel.Color c = hsvToColor(color);
            telemetry.addLine("(" + x + ", " + y + "), " + c.name() + ": " + color[0] + ", " + color[1] + ", " + color[2]);
            if (c == INVALID || c == backdrop.get(x, y).color) continue;
            backdrop.add(new Pixel(x, y, c));
        }

        for (Pixel[] row : backdrop.slots) for (Pixel pixel: row) {
            if (pixel.color != EMPTY && !backdrop.isSupported(pixel)) backdrop.add(new Pixel(pixel, EMPTY));
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

        Imgproc.line(input, center1, center2, blue, (int) (8 * SCALING_FACTOR));
        Imgproc.line(input, center2, center3, blue, (int) (8 * SCALING_FACTOR));
        Imgproc.line(input, center3, center1, blue, (int) (8 * SCALING_FACTOR));
    }

    private void warpToFitGrid(Mat input) {
        Backdrop backdrop = new Backdrop();

        Point[] target = {
                new Point(), // tl =
                new Point(), // br =
                new Point()  // bl =
        };

        for (int y = 0; y < centerPoints.length; y++) for (int x = 0; x < centerPoints[y].length; x++) {
            if (x == 0 && y % 2 == 0) continue;

            Pixel.Color color = hsvToColor(getColorOfPixel(input, y, x));
            switch (color) {
                case EMPTY: case INVALID: continue;
            }

            Pixel pixel = new Pixel(x, y, WHITE);
            if (!backdrop.isSupported(pixel)) continue;

            target[0].x = x;
            target[0].y = y;
            backdrop.add(pixel);
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

        for (Point p : new Point[]{target[0], target[1], target[2]}) {
            telemetry.addLine("(" + p.x + ", " + p.y + ")");
        }
        boolean valid = !(
                target[1].equals(target[0]) ||
                target[1].equals(target[2]) ||
                target[0].equals(target[2])
        );
        telemetry.addLine("Valid: " + valid);

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
                clip(estimate.x - boxRadius, 0, SCREEN_WIDTH),
                clip(estimate.y - boxRadius, 0, SCREEN_HEIGHT)
        );
        Mat region = warpedGray.submat(new Rect(
                topLeft,
                new Point(
                        clip(estimate.x + boxRadius, 0, SCREEN_WIDTH),
                        clip(estimate.y + boxRadius, 0, SCREEN_HEIGHT)
                )
        ));

        int blockSize = (int) (61 * SCALING_FACTOR);
        if (blockSize % 2 == 0) blockSize++;
        Imgproc.adaptiveThreshold(region, region, 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY, blockSize, -1);
        double blur = 16 * SCALING_FACTOR;
        Imgproc.blur(region, region, new Size(blur, blur));
        
        Mat circles = new Mat();
        Imgproc.HoughCircles(region, circles, Imgproc.HOUGH_GRADIENT, 1.2, 500 * SCALING_FACTOR, 1, .5, 2, -1);
        region.release();

        if (circles.size().width > 0) {

            estimate = new Point(
                    topLeft.x + circles.get(0, 0)[0],
                    topLeft.y + circles.get(0, 0)[1]
            );
        }
        circles.release();
        Imgproc.drawMarker(warpedGray, estimate, red, 2, (int) (4 * SCALING_FACTOR));
        return estimate;
    }

    private Point coordToPoint(Point p) {
        return centerPoints[(int) p.y][(int) p.x];
    }

    private void drawPixelIcon(Mat input, int y, int x) {
        int size = (int) (80 * SCALING_FACTOR);
        double sideLength = size / sqrt(3);
        int thickness = (int) (8 * SCALING_FACTOR);
        Scalar color = colorToScalar(backdrop.get(x, y).color);

        Point
                p1 = centerPoints[y][x].clone(),
                p2 = centerPoints[y][x].clone(),
                p3 = centerPoints[y][x].clone(),
                p4 = centerPoints[y][x].clone(),
                p5 = centerPoints[y][x].clone(),
                p6 = centerPoints[y][x].clone();

        p2.x += 0.5 * size;
        p2.y += 0.5 * sideLength;

        p3.x += 0.5 * size;
        p3.y -= 0.5 * sideLength;

        p5.x -= 0.5 * size;
        p5.y -= 0.5 * sideLength;

        p6.x -= 0.5 * size;
        p6.y += 0.5 * sideLength;

        p1.y += sideLength;
        p4.y -= sideLength;

        Imgproc.line(input, p1, p2, color, thickness);
        Imgproc.line(input, p2, p3, color, thickness);
        Imgproc.line(input, p3, p4, color, thickness);
        Imgproc.line(input, p4, p5, color, thickness);
        Imgproc.line(input, p5, p6, color, thickness);
        Imgproc.line(input, p6, p1, color, thickness);

        Imgproc.putText(input, x + ", " + y, p6, 2, SCALING_FACTOR, red);
    }

    private static void drawBlueSquare(Mat input, Point point) {
        if (point == null) return;
        double size = 4 * SCALING_FACTOR;
        Imgproc.rectangle(
                input,
                new Point(point.x - size, point.y - size),
                new Point(point.x + size, point.y + size),
                blue,
                1
        );
    }

    private double[] getColorOfPixel(Mat input, int y, int x) {
        double hueSum = 0, satSum = 0, valSum = 0;

        for (int i = 0; i < samplePoints[y][x].length; i++) {
            Point samplePoint = samplePoints[y][x][i];
            double[] sampleColor = input.get((int) samplePoint.y, (int) samplePoint.x);
            hueSum += sampleColor[0];
            satSum += sampleColor[1];
            valSum += sampleColor[2];
        }

        double avgHue = hueSum / ((double) samplePoints[y][x].length) * 2.0; // HUE IS MULTIPLIED BY 2 FOR RANGE [0, 360]
        double avgSat = satSum / ((double) samplePoints[y][x].length) / 255.0;
        double avgVal = valSum / ((double) samplePoints[y][x].length) / 255.0;

        return new double[]{
                avgHue,
                round(avgSat * 1000) / 1000.0,
                round(avgVal * 1000) / 1000.0,
        };
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
        double width = 97.825 * SCALING_FACTOR;
        double height = -85.0 * SCALING_FACTOR;
        for (int y = 0; y < centerPoints.length; y++) for (int x = 0; x < centerPoints[y].length; x++) {
            boolean evenRow = y % 2 == 0;
            if (x == 0 && evenRow) continue;

            centerPoints[y][x] = new Point(
                    X_FIRST_PIXEL + (x * width) - (evenRow ? 0.5 * width : 0),
                    Y_FIRST_PIXEL + (y * height)
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

    private static double getLeftX(int id) {
        return (X_TOP_LEFT_R_TAG) - ((3 - (id - (id > 3 ? 3 : 0))) * (6 * (TARGET_SIZE / 2.0)));
    }

    private Point pixelLeft(int x, int y) {
        Point point = centerPoints[y][x].clone();
        point.x += (X_SHIFT_PIXEL_POINTS_L);
        return point;
    }

    private Point pixelRight(int x, int y) {
        Point point = centerPoints[y][x].clone();
        point.x += (X_SHIFT_PIXEL_POINTS_R);
        return point;
    }

    private Point pixelTop(int x, int y) {
        Point point = centerPoints[y][x].clone();
        point.y += (Y_SHIFT_PIXEL_POINTS_T);
        return point;
    }

    private Point pixelBottom(int x, int y) {
        Point point = centerPoints[y][x].clone();
        point.y += (Y_SHIFT_PIXEL_POINTS_B);
        return point;
    }
}
