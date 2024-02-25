package org.firstinspires.ftc.teamcode.control.vision.detectors;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.isRed;
import static org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.PropDetectPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class TeamPropDetector {

    private final OpenCvCamera camera;

    public final PropDetectPipeline pipeline;

    private volatile boolean isOpen = false;

    /**
     * @param hardwareMap     {@link HardwareMap} passed in from the opmode
     */
    public TeamPropDetector(HardwareMap hardwareMap) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "camera front"),
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        );
        pipeline = new PropDetectPipeline(mTelemetry);
        pipeline.isRed = isRed;
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, UPRIGHT);
                isOpen = true;
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void printTelemetry() {
        mTelemetry.addData("Location", pipeline.getLocation().name());
    }

    public void stop() {
        while (!isOpen) {}
        camera.stopStreaming();
        camera.closeCameraDevice();
    }
}
