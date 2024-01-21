package org.firstinspires.ftc.teamcode.subsystems.utilities.sensors;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.CENTER;
import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.LEFT;
import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.RIGHT;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline;

@Config
public class TeamPropDetector {

    public static double
            MAX_LEFT_DISTANCE = 20,
            MAX_RIGHT_DISTANCE = 20;

    private PropDetectPipeline.Randomization location = CENTER;

    private final DistanceSensor leftSensor, rightSensor;

    private double leftDistance, rightDistance;

    /**
     * @param hardwareMap     {@link HardwareMap} passed in from the opmode
     */
    public TeamPropDetector(HardwareMap hardwareMap) {
        leftSensor = hardwareMap.get(DistanceSensor.class, "left distance");
        rightSensor = hardwareMap.get(DistanceSensor.class, "center distance");
    }

    public PropDetectPipeline.Randomization run() {
        return location = (
                (leftDistance = leftSensor.getDistance(INCH)) <= MAX_LEFT_DISTANCE ? LEFT :
                (rightDistance = rightSensor.getDistance(INCH)) <= MAX_RIGHT_DISTANCE ? RIGHT :
                CENTER
        );
    }

    public PropDetectPipeline.Randomization getLocation() {
        return location;
    }

    public void printNumericalTelemetry() {
        mTelemetry.addData("Left distance", leftDistance);
        mTelemetry.addData("Right distance", rightDistance);
    }
}
