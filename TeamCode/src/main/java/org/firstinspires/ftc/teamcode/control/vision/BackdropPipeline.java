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

import static org.firstinspires.ftc.teamcode.control.vision.AprilTagDetectionPipeline.draw3dCubeMarker;
import static org.firstinspires.ftc.teamcode.control.vision.AprilTagDetectionPipeline.drawAxisMarker;
import static org.firstinspires.ftc.teamcode.control.vision.AprilTagDetectionPipeline.poseFromTrapezoid;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    public boolean warp = true, backdropVisible = false, isRed = true;

    public double
            X_TOP_LEFT_R_TAG = 650,
            Y_TOP_LEFT = 1300,
            TARGET_SIZE = 85;

    private long nativeApriltagPtr;
    private final Mat grey = new Mat(), cameraMatrix;

    public final Scalar[][] backdrop = new Scalar[11][7];

    private final double
            fx = 1430,
            fy = 1430,
            cx = 480,
            cy = 620,
            tagSize = 0.0508;

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

        // Run AprilTag
        ArrayList<AprilTagDetection> detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagSize, fx, fy, cx, cy);

        // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
        // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
        for (AprilTagDetection detection : detections) {
            AprilTagDetectionPipeline.Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagSize, tagSize);
            drawAxisMarker(input, tagSize / 2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
            draw3dCubeMarker(input, tagSize, tagSize, tagSize, 5, pose.rvec, pose.tvec, cameraMatrix);
        }

        AprilTagDetection tag = null;
        allTags:
        for (AprilTagDetection detection : detections) {
            if (isRed) {
                switch (detection.id) {
                    case 4: case 5: case 6:
                        tag = detection;
                        break allTags;
                }
            } else {
                switch (detection.id) {
                    case 1: case 2: case 3:
                        tag = detection;
                        break allTags;
                }
            }
        }

        backdropVisible = tag != null;

        if (warp && backdropVisible) {
            telemetry.addData("ID", tag.id);

            Point bl = tag.corners[0];
            Point br = tag.corners[1];
            Point tr = tag.corners[2];
            Point tl = tag.corners[3];

            MatOfPoint2f srcTag = new MatOfPoint2f(
                    bl,
                    br,
                    tr,
                    tl
            );

            int id2 = tag.id - (tag.id > 3 ? 3 : 0);
            double topLeftX = X_TOP_LEFT_R_TAG - ((3 - id2) * (6 * (TARGET_SIZE / 2.0)));

            Point
                    rightTagTR = new Point(topLeftX + TARGET_SIZE, Y_TOP_LEFT),
                    rightTagBL = new Point(topLeftX, Y_TOP_LEFT + TARGET_SIZE),
                    rightTagTL = new Point(topLeftX, Y_TOP_LEFT),
                    rightTagBR = new Point(topLeftX + TARGET_SIZE, Y_TOP_LEFT + TARGET_SIZE);

            MatOfPoint2f dstTag = new MatOfPoint2f(
                    rightTagBL,
                    rightTagBR,
                    rightTagTR,
                    rightTagTL
            );

            Mat transformMatrix = Imgproc.getPerspectiveTransform(srcTag, dstTag);

            Imgproc.warpPerspective(input, input, transformMatrix, input.size());
        }
        telemetry.update();

        return input;
    }
}