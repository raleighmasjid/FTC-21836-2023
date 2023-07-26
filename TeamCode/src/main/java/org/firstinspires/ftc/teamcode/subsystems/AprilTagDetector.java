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
     * Prints tag visibility status to telemetry and calls {@link #printLastDetection()} <p>
     * telemetry.update() should be called after this method
     */
    public void run() {
        tagVisible = checkIfTagDetected(pipeline.getLatestDetections());
        myTelemetry.addLine("A tag of interest is " + (tagVisible ? "" : "not ") + "visible");
        printLastDetection();
    }

    /**
     * Closes the camera and calls {@link #printLastDetection()} <p>
     * telemetry.update() should be called after this method
     */
    public void stop() {
        camera.stopStreaming();
        camera.closeCameraDevice();
        printLastDetection();
    }

    /**
     * Prints last {@link #detectedTag} to telemetry <p>
     * telemetry.update() should be called after this method
     */
    public void printLastDetection() {
        myTelemetry.addLine("A tag has" + (
                detectedTag == null ?
                        "never been detected" :
                        "been detected: " + detectedTag.id
        ));
    }

    /**
     * @param detections {@link AprilTagDetection} ArrayList grabbed from an {@link AprilTagDetectionPipeline}
     * @return Whether or not a {@link #detectedTag}'s id is one of the {@link #tagIdsToLookFor}
     */
    private boolean checkIfTagDetected(ArrayList<AprilTagDetection> detections) {
        if (detections.size() == 0) return false;
        for (AprilTagDetection detection : detections) {
            for (int tagId : tagIdsToLookFor) {
                if (detection.id == tagId) {
                    detectedTag = detection;
                    return true;
                }
            }
        }
        return false;
    }

    public AprilTagDetection getDetectedTag() {
        return detectedTag;
    }

    public boolean tagIsVisible() {
        return tagVisible;
    }
}
