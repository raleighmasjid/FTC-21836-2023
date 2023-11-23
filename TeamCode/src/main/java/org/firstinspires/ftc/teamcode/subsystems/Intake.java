package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1620;
import static org.firstinspires.ftc.teamcode.control.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeState.HAS_0;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeState.TRANSFERRING;
import static org.firstinspires.ftc.teamcode.subsystems.SimpleServoPivot.getAxonMini;
import static org.firstinspires.ftc.teamcode.subsystems.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.SimpleServoPivot.getReversedServo;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.placementalg.Pixel;

public class Intake {

    public static double
            ANGLE_PIVOT_OFFSET = 0,
            ANGLE_LATCH_OPEN = 0,
            ANGLE_LATCH_CLOSED = 30,
            kV = 1,
            TIME_PIVOTING = 1,
            COLOR_SENSOR_GAIN = 1;

    private final MotorEx motor;
    private final VoltageSensor batteryVoltageSensor;

    private final ThreadedColorSensor bottomSensor, topSensor;
    private float[]
            bottomHSV = new float[3],
            topHSV = new float[3];

    private final SimpleServoPivot pivot, latch;

    private IntakeState currentState = HAS_0;
    private final ElapsedTime timer = new ElapsedTime();
    private final Pixel[] pixels = new Pixel[2];
    public boolean justDroppedPixels = false;

    enum IntakeState {
        HAS_0,
        HAS_1,
        PIVOTING,
        TRANSFERRING,
    }

    public Intake(HardwareMap hardwareMap) {
        bottomSensor = new ThreadedColorSensor(hardwareMap, "bottom color", (float) COLOR_SENSOR_GAIN);
        topSensor = new ThreadedColorSensor(hardwareMap, "top color", (float) COLOR_SENSOR_GAIN);

        pivot = new SimpleServoPivot(
                new SimpleServo[]{
                        getAxonMini(hardwareMap, "intake right"),
                        getReversedServo(getAxonMini(hardwareMap, "intake left"))
                },
                ANGLE_PIVOT_OFFSET,
                ANGLE_PIVOT_OFFSET + 180
        );

        latch = new SimpleServoPivot(
                new SimpleServo[]{
                        getGoBildaServo(hardwareMap, "latch right"),
                        getReversedServo(getGoBildaServo(hardwareMap, "latch left"))
                },
                ANGLE_LATCH_OPEN,
                ANGLE_LATCH_CLOSED
        );

        motor = new MotorEx(hardwareMap, "intake", RPM_1620);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        timer.reset();
    }

    public void start() {
        bottomSensor.start();
        topSensor.start();
    }

    public void runMotor(double output) {
        motor.set(output * kV * (12.0 / batteryVoltageSensor.getVoltage()));
    }

    public void run() {

        switch (currentState) {
            case HAS_0:
                if (justDroppedPixels) justDroppedPixels = false;
                bottomHSV = bottomSensor.getHSV();
                break;
            case HAS_1:
                topHSV = topSensor.getHSV();
                break;
            case PIVOTING:
                if (timer.seconds() >= TIME_PIVOTING) {
                    currentState = TRANSFERRING;
                    latch.setActivated(false);
                }
                break;
            case TRANSFERRING:
                justDroppedPixels = Pixel.Color.fromHSV(topSensor.getHSV()) == EMPTY;
                if (justDroppedPixels) {
                    currentState = HAS_0;
                    pivot.setActivated(false);
                }
                break;
        }

        pivot.run();
        latch.run();
    }

    public void interrupt() {
        bottomSensor.interrupt();
        topSensor.interrupt();
    }

    public Pixel[] getPixels() {
        return pixels;
    }

    private void printHSV(MultipleTelemetry telemetry, float[] hsv, String title) {
        telemetry.addLine(title + ":");
        telemetry.addData("Hue", hsv[0]);
        telemetry.addData("Saturation", hsv[1]);
        telemetry.addData("Value", hsv[2]);
    }

    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        printHSV(telemetry, bottomHSV, "Bottom HSV");
        telemetry.addLine();
        printHSV(telemetry, topHSV, "Top HSV");
    }
}