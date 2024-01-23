package org.firstinspires.ftc.teamcode.control.vision;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRed;
import static org.openftc.easyopencv.OpenCvCameraRotation.SIDEWAYS_RIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Config
public class BackdropDetector {

    private final OpenCvCamera camera;

    private final BackdropPipeline pipeline;

    public final Backdrop backdrop = new Backdrop();

    private volatile boolean isOpen = false;

    /**
     * @param hardwareMap     {@link HardwareMap} passed in from the opmode
     */
    public BackdropDetector(HardwareMap hardwareMap) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "camera front"),
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        );
        pipeline = new BackdropPipeline(mTelemetry);
        pipeline.isRed = isRed;
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(960, 1280, SIDEWAYS_RIGHT);
                isOpen = true;
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public Backdrop run() {

        int[][] slots = pipeline.slots;
        for (int y = 0; y < slots.length; y++) for (int x = 0; x < slots[y].length; x++) {
            if (x == 0 && y % 2 == 0) continue;
            backdrop.add(new Pixel(x, y, Pixel.Color.get(slots[y][x])));
        }

        return backdrop;
    }

    /**
     * Closes the camera
     */
    public void stop() {
        while (!isOpen) {}
        camera.stopStreaming();
        camera.closeCameraDevice();
    }
}
