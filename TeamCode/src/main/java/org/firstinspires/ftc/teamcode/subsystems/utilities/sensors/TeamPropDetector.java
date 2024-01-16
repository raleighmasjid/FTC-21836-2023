package org.firstinspires.ftc.teamcode.subsystems.utilities.sensors;

import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.RIGHT;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRed;
import static org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Config
public class TeamPropDetector {

    private PropDetectPipeline.Randomization location = RIGHT;

    private final OpenCvCamera camera;

    private final PropDetectPipeline pipeline;

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
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void run() {
        location = pipeline.getLocation();
    }

    public PropDetectPipeline.Randomization getLocation() {
        return location;
    }

    /**
     * Closes the camera
     */
    public void stop() {
        camera.stopStreaming();
        camera.closeCameraDevice();
    }
}
