package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.control.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
public class AprilTagDetector {

    public static double
            TAG_SIZE = 0.166,
            CAMERA_FX = 578.272,
            CAMERA_FY = 578.272,
            CAMERA_CX = 402.145,
            CAMERA_CY = 221.506;

    private AprilTagDetection detectedTag = null;

    private final OpenCvCamera camera;

    private final AprilTagDetectionPipeline pipeline = new AprilTagDetectionPipeline(
            TAG_SIZE,
            CAMERA_FX,
            CAMERA_FY,
            CAMERA_CX,
            CAMERA_CY
    );

    private final MultipleTelemetry myTelemetry;

    private final int[] tagIdsToLookFor;

    private boolean tagVisible = false;

    /**
     * @param hardwareMap     {@link HardwareMap} passed in from the opmode
     * @param myTelemetry     {@link MultipleTelemetry} telemetry to print output to
     * @param tagIdsToLookFor integer IDs of April Tags to look for
     * @param cameraRotation  physical orientation of camera
     */
    public AprilTagDetector(HardwareMap hardwareMap, MultipleTelemetry myTelemetry, int[] tagIdsToLookFor, OpenCvCameraRotation cameraRotation) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        );
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, cameraRotation);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        this.myTelemetry = myTelemetry;
        this.tagIdsToLookFor = tagIdsToLookFor;
    }

    /**
     * Gets detections from pipeline<p>
     * Use {@link #getTagIsVisible()} ()} and {@link #getDetectedTag()}
     */
    public void run() {
        ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();
        if (detections.size() == 0) {
            tagVisible = false;
            return;
        }
        for (AprilTagDetection detection : detections) {
            for (int tagId : tagIdsToLookFor) {
                if (detection.id == tagId) {
                    detectedTag = detection;
                    tagVisible = true;
                    return;
                }
            }
        }
        tagVisible = false;
    }

    public boolean getTagIsVisible() {
        return tagVisible;
    }

    public AprilTagDetection getDetectedTag() {
        return detectedTag;
    }

    /**
     * Prints tag visibility to telemetry <p>
     * telemetry.update() should be called after this method
     */
    public void printTagIsVisible() {
        myTelemetry.addLine("A tag of interest is " + (getTagIsVisible() ? "" : "not ") + "visible");
    }

    /**
     * Prints last {@link #detectedTag} to telemetry <p>
     * telemetry.update() should be called after this method
     */
    public void printDetectedTag() {
        AprilTagDetection detectedTag = getDetectedTag();
        myTelemetry.addLine("A tag has" + (
                detectedTag == null ?
                        "never been detected" :
                        "been detected: " + detectedTag.id
        ));
    }

    /**
     * Closes the camera
     */
    public void stop() {
        camera.stopStreaming();
        camera.closeCameraDevice();
    }
}
