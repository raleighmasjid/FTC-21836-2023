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

package org.firstinspires.ftc.teamcode.control.vision;

import static org.firstinspires.ftc.teamcode.control.vision.AprilTagDetectionPipeline.blue;
import static org.firstinspires.ftc.teamcode.control.vision.AprilTagDetectionPipeline.draw3dCubeMarker;
import static org.firstinspires.ftc.teamcode.control.vision.AprilTagDetectionPipeline.drawAxisMarker;
import static org.firstinspires.ftc.teamcode.control.vision.AprilTagDetectionPipeline.poseFromTrapezoid;
import static org.firstinspires.ftc.teamcode.control.vision.AprilTagDetectionPipeline.yellow;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;

public class BackdropPipeline extends OpenCvPipeline {

    public boolean warp = true, backdropVisible = false, isRed = true;

    public double
            X_TOP_LEFT_R_TAG = 670,
            Y_TOP_LEFT = 1100,
            TARGET_SIZE = 75,
            X_SHIFT_L_TAG_TO_L_PIXEL = -115,
            Y_SHIFT_TAG_TO_PIXEL = -100,
            X_SHIFT_PIXEL_POINTS = 80;

    private final ArrayList<AprilTagDetection> tags = new ArrayList<>();

    private long nativeApriltagPtr;
    private final Mat grey = new Mat(), cameraMatrix;

    public final int[][] slots = new int[11][7];
    private final Point[][][] points = new Point[11][7][2];

    private final double
            fx = 1430,
            fy = 1430,
            cx = 480,
            cy = 620,
            tagSize = 0.0508;

    public BackdropPipeline(Telemetry telemetry) {

        for (int[] row : slots) Arrays.fill(row, 4);

        for (int y = 0; y < points.length; y++) {
            for (int x = 0; x < points[y].length; x++) {
                if (x == 0 && y % 2 == 0) continue;
                points[y][x][0] = pixelLeft(x, y);
                points[y][x][1] = pixelRight(x, y);
            }
        }

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
        int center = !isRed ? 2 : 5;
        int right = !isRed ? 3 : 6;

        for (AprilTagDetection detection : detections) {
            int id = detection.id;
            if (id == left || id == center || id == right) {
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

            Imgproc.line(input, tl, tr, blue, 5);
            Imgproc.line(input, bl, br, blue, 5);
            Imgproc.line(input, tl, bl, blue, 5);
            Imgproc.line(input, tr, br, blue, 5);

            if (warp) {
                Mat transformMatrix = Imgproc.getPerspectiveTransform(srcTag, dstTag);
                Imgproc.warpPerspective(input, input, transformMatrix, input.size());

                for (Point[][] row : points) for (Point[] pair : row) for (Point point : pair) {
                    if (point != null) Imgproc.drawMarker(input, point, blue, 1, 2, 7);
                }
            }

            Imgproc.line(input, tagTL, tagTR, yellow, 5);
            Imgproc.line(input, tagBL, tagBR, yellow, 5);
            Imgproc.line(input, tagTL, tagBL, yellow, 5);
            Imgproc.line(input, tagTR, tagBR, yellow, 5);
        }

        String tagIds = "";
        for (AprilTagDetection tag : tags) tagIds += tag.id + " ";
        telemetry.addData("Detected tags", tagIds);
        telemetry.update();

        return input;
    }

    private double getLeftX(int id) {
        return X_TOP_LEFT_R_TAG - ((3 - (id - (id > 3 ? 3 : 0))) * (6 * (TARGET_SIZE / 2.0)));
    }

    private Point pixelLeft(int x, int y) {
        double width = 2.985 * (TARGET_SIZE / 2.0);
        return new Point(
                getLeftX(1) + X_SHIFT_L_TAG_TO_L_PIXEL + (x * width) - (y % 2 == 0 ? 0.5 * width : 0),
                Y_TOP_LEFT + Y_SHIFT_TAG_TO_PIXEL - y * (2.53 * (TARGET_SIZE / 2.0))
        );
    }

    private Point pixelRight(int x, int y) {
        Point left = pixelLeft(x, y);
        left.x += X_SHIFT_PIXEL_POINTS;
        return left;
    }
}