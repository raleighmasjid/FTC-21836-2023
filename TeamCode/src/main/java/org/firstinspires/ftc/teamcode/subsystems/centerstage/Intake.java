package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1620;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.HAS_1_PIXEL;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.PIVOTING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.HAS_0_PIXELS;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.TRANSFERRING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FLOOR;
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
            ANGLE_PIVOT_CLEARANCE = 3,
            ANGLE_LATCH_OPEN = 0,
            ANGLE_LATCH_CLOSED = 30,
            TIME_REVERSING = 1,
            TIME_PIVOTING = 1,
            COLOR_SENSOR_GAIN = 1,
            COLOR_VALUE_MULTIPLIER;

    private final MotorEx motor;

    private final ThreadedColorSensor bottomSensor, topSensor;
    private HSV bottomHSV = new HSV(), topHSV = new HSV();

    private final SimpleServoPivot pivot, latch;

    private State state = HAS_0_PIXELS;
    private Height height = FLOOR;
    private final ElapsedTime timer = new ElapsedTime();
    private final Pixel.Color[] colors = {EMPTY, EMPTY};
    private boolean justDroppedPixels = false;

    enum State {
        HAS_0_PIXELS,
        HAS_1_PIXEL,
        PIVOTING,
        TRANSFERRING,
    }

    public enum Height {
        FLOOR,
        TWO_STACK,
        THREE_STACK,
        FOUR_STACK,
        FIVE_STACK;

        private final double deltaX, deltaTheta;

        private static final Height[] values = values();
        private static Height get(int ordinal) {
            return values[ordinal];
        }

        Height() {
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

        pivot = new SimpleServoPivot(
                ANGLE_PIVOT_OFFSET,
                ANGLE_PIVOT_OFFSET + 180,
                getReversedServo(getAxonServo(hardwareMap, "intake right")),
                getAxonServo(hardwareMap, "intake left")
        );

        latch = new SimpleServoPivot(
                ANGLE_LATCH_OPEN,
                ANGLE_LATCH_CLOSED,
                getGoBildaServo(hardwareMap, "latch right"),
                getReversedServo(getGoBildaServo(hardwareMap, "latch left"))
        );

        motor = new MotorEx(hardwareMap, "intake", RPM_1620);
        motor.setZeroPowerBehavior(FLOAT);
        motor.setInverted(true);

        bottomSensor = new ThreadedColorSensor(hardwareMap, "bottom color", (float) COLOR_SENSOR_GAIN);
        topSensor = new ThreadedColorSensor(hardwareMap, "top color", (float) COLOR_SENSOR_GAIN);

        timer.reset();
    }

    void interrupt() {
        bottomSensor.interrupt();
        topSensor.interrupt();
    }

    public void setMotorPower(double motorPower) {
        motor.set(motorPower);
    }

    void run() {

        if (justDroppedPixels) justDroppedPixels = false;

        switch (state) {
            case HAS_0_PIXELS:
                bottomHSV = bottomSensor.getHSV();
                colors[0] = Pixel.Color.fromHSV(bottomHSV);
//                if (!colors[0].isEmpty()) {
//                    state = HAS_1_PIXEL;
//                    decrementHeight();
//                }
//                break;
//            case HAS_1_PIXEL:
                topHSV = topSensor.getHSV();
                colors[1] = Pixel.Color.fromHSV(topHSV);
//                if (!colors[1].isEmpty()) {
//                    state = PIVOTING;
//                    decrementHeight();
//                    timer.reset();
//                    latch.setActivated(true);
//                    pivot.setActivated(true);
//                }
                break;
            case PIVOTING:
                if (timer.seconds() <= TIME_REVERSING) setMotorPower(-1);
                if (timer.seconds() >= TIME_PIVOTING) {
                    setMotorPower(0);
                    state = TRANSFERRING;
                    latch.setActivated(false);
                }
                break;
            case TRANSFERRING:
                justDroppedPixels = Pixel.Color.fromHSV(topSensor.getHSV()).isEmpty() && Pixel.Color.fromHSV(bottomSensor.getHSV()).isEmpty();
                if (justDroppedPixels) {
                    state = HAS_0_PIXELS;
                    pivot.setActivated(false);
                }
                break;
        }

        pivot.updateAngles((motor.get() > 0 ? 0 : ANGLE_PIVOT_CLEARANCE) + ANGLE_PIVOT_OFFSET + height.deltaTheta, ANGLE_PIVOT_OFFSET + 180);
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
        setHeight(Height.get(max(height.ordinal() - 1, 0)));
    }

    public void setHeight(Height height) {
        this.height = height;
    }

    public double getStackXOffset() {
        return height.deltaX;
    }

    private void printHSV(MultipleTelemetry telemetry, HSV color, String title) {
        telemetry.addLine(title + ":");
        telemetry.addData("Hue", color.hue);
        telemetry.addData("Saturation", color.saturation);
        telemetry.addData("Value", color.value);
    }

    void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Bottom color", colors[0].name());
        telemetry.addData("Top color", colors[1].name());
    }

    void printNumericalTelemetry(MultipleTelemetry telemetry) {
        printHSV(telemetry, bottomHSV, "Bottom HSV");
        telemetry.addLine();
        printHSV(telemetry, topHSV, "Top HSV");
    }
}