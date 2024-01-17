package org.firstinspires.ftc.teamcode.subsystems.utilities.sensors;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.CENTER;
import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.LEFT;
import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.RIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline;

@Config
public class TeamPropDetector {

    public static double
            MAX_LEFT_DISTANCE = 20,
            MAX_CENTER_DISTANCE = 20;

    private PropDetectPipeline.Randomization location = RIGHT;

    private final DistanceSensor leftSensor, centerSensor;

    /**
     * @param hardwareMap     {@link HardwareMap} passed in from the opmode
     */
    public TeamPropDetector(HardwareMap hardwareMap) {
        leftSensor = hardwareMap.get(DistanceSensor.class, "left distance");
        centerSensor = hardwareMap.get(DistanceSensor.class, "center distance");
    }

    public PropDetectPipeline.Randomization run() {
        return location = (
                leftSensor.getDistance(INCH) <= MAX_LEFT_DISTANCE ? LEFT :
                centerSensor.getDistance(INCH) <= MAX_CENTER_DISTANCE ? CENTER :
                RIGHT
        );
    }

    public PropDetectPipeline.Randomization getLocation() {
        return location;
    }
}
