package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1620;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.control.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.control.placementalg.Pixel.Color.fromHSV;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeState.HAS_0_PIXELS;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeState.PIVOTING;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeState.TRANSFERRING;
import static org.firstinspires.ftc.teamcode.subsystems.SimpleServoPivot.getAxonMini;
import static org.firstinspires.ftc.teamcode.subsystems.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.SimpleServoPivot.getReversedServo;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
            TIME_REVERSING = 1,
            TIME_PIVOTING = 1,
            COLOR_SENSOR_GAIN = 1;

    private final HardwareMap hardwareMap;
    private final MotorEx motor;

    private ThreadedColorSensor bottomSensor, topSensor;
    private float[]
            bottomHSV = new float[3],
            topHSV = new float[3];

    private final SimpleServoPivot pivot, latch;

    private IntakeState currentState = HAS_0_PIXELS;
    private final ElapsedTime timer = new ElapsedTime();
    private final Pixel.Color[] colors = new Pixel.Color[]{
            EMPTY, EMPTY
    };
    private boolean justDroppedPixels = false;

    enum IntakeState {
        HAS_0_PIXELS,
        HAS_1_PIXELS,
        REVERSING,
        PIVOTING,
        TRANSFERRING,
    }

    public Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        pivot = new SimpleServoPivot(
                ANGLE_PIVOT_OFFSET,
                ANGLE_PIVOT_OFFSET + 180,
                getAxonMini(hardwareMap, "intake right"),
                getReversedServo(getAxonMini(hardwareMap, "intake left"))
        );

        latch = new SimpleServoPivot(
                ANGLE_LATCH_OPEN,
                ANGLE_LATCH_CLOSED,
                getGoBildaServo(hardwareMap, "latch right"),
                getReversedServo(getGoBildaServo(hardwareMap, "latch left"))
        );

        motor = new MotorEx(hardwareMap, "intake", RPM_1620);
        motor.setZeroPowerBehavior(FLOAT);
        timer.reset();
    }

    public void start() {
        bottomSensor = new ThreadedColorSensor(hardwareMap, "bottom color", (float) COLOR_SENSOR_GAIN);
        topSensor = new ThreadedColorSensor(hardwareMap, "top color", (float) COLOR_SENSOR_GAIN);
    }

    public void setMotorPower(double motorPower) {
        motor.set(motorPower);
    }

    public void run () {

        if (justDroppedPixels) justDroppedPixels = false;

        switch (currentState) {
            case HAS_0_PIXELS:
                bottomHSV = bottomSensor.getHSV();
                /* if (fromHSV(bottomHSV) != EMPTY) {
                    currentState = HAS_1;
                    colors[0] = fromHSV(bottomHSV);
                }
                break; */
            case HAS_1_PIXELS:
                topHSV = topSensor.getHSV();
                /* if (fromHSV(topHSV) != EMPTY) {
                    currentState = REVERSING;
                    timer.reset();
                    colors[1] = fromHSV(topHSV);
                    latch.setActivated(true);
                } */
                break;
            case REVERSING:
                setMotorPower(-1);
                if (timer.seconds() >= TIME_REVERSING) {
                    currentState = PIVOTING;
                    pivot.setActivated(true);
                }
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
                    currentState = HAS_0_PIXELS;
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

    public Pixel.Color[] getColors() {
        return colors;
    }

    public boolean justDroppedPixels() {
        return justDroppedPixels;
    }

    public IntakeState getCurrentState() {
        return currentState;
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