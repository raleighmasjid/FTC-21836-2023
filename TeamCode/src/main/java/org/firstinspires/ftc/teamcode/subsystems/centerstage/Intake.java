package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1620;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.IntakeState.HAS_1_PIXEL;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.IntakeState.PIVOTING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.IntakeState.HAS_0_PIXELS;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.IntakeState.TRANSFERRING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.IntakingHeight.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getAxonServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;
import static java.lang.Math.asin;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystems.utilities.ThreadedColorSensor;

@Config
public final class Intake {

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
    private HSV bottomHSV = new HSV(), topHSV = new HSV();

    private final SimpleServoPivot pivot, latch;

    private IntakeState currentState = HAS_0_PIXELS;
    private IntakingHeight intakingHeight = FLOOR;
    private final ElapsedTime timer = new ElapsedTime();
    private final Pixel.Color[] colors = {EMPTY, EMPTY};
    private boolean justDroppedPixels = false;

    enum IntakeState {
        HAS_0_PIXELS,
        HAS_1_PIXEL,
        PIVOTING,
        TRANSFERRING,
    }

    public enum IntakingHeight {
        FLOOR,
        TWO_STACK,
        THREE_STACK,
        FOUR_STACK,
        FIVE_STACK;

        private final double deltaX, deltaTheta;

        private static final IntakingHeight[] values = values();
        static IntakingHeight get(int ordinal) {
            return values[ordinal];
        }

        IntakingHeight() {
            if (ordinal() == 0) {
                deltaTheta = 0;
                deltaX = 0;
                return;
            }

            double r = 9.5019488189;
            double theta0 = -0.496183876745;
            double y0 = -4.523622;
            double x0 = 8.35606811024;

            double deltaY = ordinal() * 0.5 - 0.1;

            double theta1 = asin((y0 + deltaY) / r);
            deltaTheta = toDegrees(theta1 - theta0);
            deltaX = r * cos(theta1) - x0;
        }
    }


    Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        pivot = new SimpleServoPivot(
                ANGLE_PIVOT_OFFSET,
                ANGLE_PIVOT_OFFSET + 180,
                getAxonServo(hardwareMap, "intake right"),
                getReversedServo(getAxonServo(hardwareMap, "intake left"))
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

    void start() {
        bottomSensor = new ThreadedColorSensor(hardwareMap, "bottom color", (float) COLOR_SENSOR_GAIN);
        topSensor = new ThreadedColorSensor(hardwareMap, "top color", (float) COLOR_SENSOR_GAIN);
    }

    void interrupt() {
        bottomSensor.interrupt();
        topSensor.interrupt();
    }

    public void setMotorPower(double motorPower) {
        motor.set(motorPower);
    }

    void run () {

        if (justDroppedPixels) justDroppedPixels = false;

        switch (currentState) {
            case HAS_0_PIXELS:
                bottomHSV = bottomSensor.getHSV();
                colors[0] = Pixel.Color.fromHSV(bottomHSV);
                if (!colors[0].isEmpty()) {
                    currentState = HAS_1_PIXEL;
                    decrementHeight();
                }
                break;
            case HAS_1_PIXEL:
                topHSV = topSensor.getHSV();
                colors[1] = Pixel.Color.fromHSV(topHSV);
                if (!colors[1].isEmpty()) {
                    currentState = PIVOTING;
                    decrementHeight();
                    timer.reset();
                    latch.setActivated(true);
                    pivot.setActivated(true);
                }
                break;
            case PIVOTING:
                if (timer.seconds() <= TIME_REVERSING) setMotorPower(-1);
                if (timer.seconds() >= TIME_PIVOTING) {
                    setMotorPower(0);
                    currentState = TRANSFERRING;
                    latch.setActivated(false);
                }
                break;
            case TRANSFERRING:
                justDroppedPixels = Pixel.Color.fromHSV(topSensor.getHSV()).isEmpty() && Pixel.Color.fromHSV(bottomSensor.getHSV()).isEmpty();
                if (justDroppedPixels) {
                    currentState = HAS_0_PIXELS;
                    pivot.setActivated(false);
                }
                break;
        }

        pivot.updateAngles(ANGLE_PIVOT_OFFSET + intakingHeight.deltaTheta, ANGLE_PIVOT_OFFSET + 180);
        latch.updateAngles(ANGLE_LATCH_OPEN, ANGLE_LATCH_CLOSED);

        pivot.run();
        latch.run();
    }

    Pixel.Color[] getColors() {
        return colors;
    }

    int pixelCount() {
        return (colors[0].isEmpty() ? 0 : 1) + (colors[1].isEmpty() ? 0 : 1);
    }

    boolean justDroppedPixels() {
        return justDroppedPixels;
    }

    private void decrementHeight() {
        setHeight(IntakingHeight.get(max(intakingHeight.ordinal() - 1, 0)));
    }

    public void setHeight(IntakingHeight intakingHeight) {
        this.intakingHeight = intakingHeight;
    }

    public double getStackXOffset() {
        return intakingHeight.deltaX;
    }

    private void printHSV(MultipleTelemetry telemetry, HSV color, String title) {
        telemetry.addLine(title + ":");
        telemetry.addData("Hue", color.hue);
        telemetry.addData("Saturation", color.saturation);
        telemetry.addData("Value", color.value);
    }

    void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Bottom color", colors[0].toString());
        telemetry.addData("Top color", colors[1].toString());
    }

    void printNumericalTelemetry(MultipleTelemetry telemetry) {
        printHSV(telemetry, bottomHSV, "Bottom HSV");
        telemetry.addLine();
        printHSV(telemetry, topHSV, "Top HSV");
    }
}