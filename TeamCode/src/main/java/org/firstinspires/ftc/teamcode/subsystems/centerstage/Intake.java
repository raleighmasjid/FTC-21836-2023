package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1620;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.HAS_0_PIXELS;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.HAS_1_PIXEL;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.PIVOTING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.PIXELS_SETTLING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.PIXELS_FALLING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;
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
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystems.utilities.ThreadedColorSensor;

@Config
public final class Intake {

    public static double
            ANGLE_PIVOT_OFFSET = 6,
            ANGLE_PIVOT_TRANSFERRING = 195,
            ANGLE_FLOOR_CLEARANCE = 4,
            ANGLE_LATCH_TRANSFERRING = 0,
            ANGLE_LATCH_INTAKING = 50,
            ANGLE_LATCH_LOCKED = 149,
            TIME_REVERSING = 1,
            TIME_PIVOTING = 5,
            COLOR_SENSOR_GAIN = 1,
            SPEED_SLOW_REVERSING = -0.25,
            SETTLING_TIME = 1;

    private final MotorEx motor;

    private final ThreadedColorSensor bottomSensor, topSensor;
    private HSV bottomHSV = new HSV(), topHSV = new HSV();

    private final TouchSensor pivotSensor;

    private final SimpleServoPivot pivot, latch;

    private Intake.State state = HAS_0_PIXELS;
    private Intake.Height height = FLOOR;

    private final ElapsedTime timer = new ElapsedTime();
    private final Pixel.Color[] colors = {EMPTY, EMPTY};

    private boolean pixelsTransferred = false;
    private int requiredPixelCount = 2;
    private double motorPower = 0;

    enum State {
        HAS_0_PIXELS,
        HAS_1_PIXEL,
        PIVOTING,
        PIXELS_FALLING,
        PIXELS_SETTLING,
    }

    public enum Height {
        FLOOR,
        TWO_STACK,
        THREE_STACK,
        FOUR_STACK,
        FIVE_STACK;

        private final double deltaX, deltaTheta;

        private static final Intake.Height[] values = values();
        private static Intake.Height get(int ordinal) {
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
                ANGLE_PIVOT_OFFSET + ANGLE_PIVOT_TRANSFERRING,
                getAxonServo(hardwareMap, "intake right"),
                getReversedServo(getAxonServo(hardwareMap, "intake left"))
        );

        latch = new SimpleServoPivot(
                ANGLE_LATCH_TRANSFERRING,
                ANGLE_LATCH_LOCKED,
                getGoBildaServo(hardwareMap, "latch right"),
                getReversedServo(getGoBildaServo(hardwareMap, "latch left"))
        );

        motor = new MotorEx(hardwareMap, "intake", RPM_1620);
        motor.setZeroPowerBehavior(FLOAT);
        motor.setInverted(true);

        bottomSensor = new ThreadedColorSensor(hardwareMap, "bottom color", (float) COLOR_SENSOR_GAIN);
        topSensor = new ThreadedColorSensor(hardwareMap, "top color", (float) COLOR_SENSOR_GAIN);

        pivotSensor = hardwareMap.get(TouchSensor.class, "intake pivot sensor");

        timer.reset();
    }

    void interrupt() {
        bottomSensor.interrupt();
        topSensor.interrupt();
    }

    public void setMotorPower(double motorPower) {
        this.motorPower = motorPower;
    }

    public void setRequiredPixelCount(int pixelCount) {
        this.requiredPixelCount = clip(pixelCount, 0, 2);
    }

    void run() {

        if (pixelsTransferred) pixelsTransferred = false;

        switch (state) {
            case HAS_0_PIXELS:

                bottomHSV = bottomSensor.getHSV();
                colors[0] = Pixel.Color.fromHSV(bottomHSV);
                boolean bottomFull = !colors[0].isEmpty();
                if (bottomFull || requiredPixelCount == 0) {
                    if (bottomFull) decrementHeight();
                    state = HAS_1_PIXEL;
                }
                break;

            case HAS_1_PIXEL:

                topHSV = topSensor.getHSV();
                colors[1] = Pixel.Color.fromHSV(topHSV);
                boolean topFull = !colors[1].isEmpty();
                if (topFull || requiredPixelCount <= 1) {
                    if (topFull) decrementHeight();
                    timer.reset();
                    latch.setActivated(true);
                    pivot.setActivated(true);
                    state = PIVOTING;
                }
                break;

            case PIVOTING:

                if (pivotSensor.isPressed()) {
                    setMotorPower(0);
                    state = PIXELS_FALLING;
                    latch.setActivated(false);
                }
                else if (timer.seconds() <= TIME_REVERSING) setMotorPower(-1);
                else setMotorPower(SPEED_SLOW_REVERSING);
                break;

            case PIXELS_FALLING:

                if (Pixel.Color.fromHSV(topSensor.getHSV()).isEmpty() && Pixel.Color.fromHSV(bottomSensor.getHSV()).isEmpty() && requiredPixelCount > 0) {
                    state = PIXELS_SETTLING;
                    timer.reset();
                }
                break;

            case PIXELS_SETTLING:

                pixelsTransferred = timer.seconds() >= SETTLING_TIME;
                if (pixelsTransferred) {
                    state = HAS_0_PIXELS;
                    pivot.setActivated(false);
                }
                break;

        }

        pivot.updateAngles((motorPower > 0 ? 0 : ANGLE_FLOOR_CLEARANCE) + ANGLE_PIVOT_OFFSET + height.deltaTheta, ANGLE_PIVOT_OFFSET + ANGLE_PIVOT_TRANSFERRING);
        latch.updateAngles((state == HAS_0_PIXELS || state == HAS_1_PIXEL ? ANGLE_LATCH_INTAKING : ANGLE_LATCH_TRANSFERRING), ANGLE_LATCH_LOCKED);

        pivot.run();
        latch.run();

        motor.set(motorPower);
    }

    Pixel.Color[] getColors() {
        return colors.clone();
    }

    boolean pixelsTransferred() {
        return pixelsTransferred;
    }

    private void decrementHeight() {
        setHeight(Intake.Height.get(max(height.ordinal() - 1, 0)));
    }

    public void setHeight(Intake.Height height) {
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
        telemetry.addData("Top color", colors[1].name());
        telemetry.addData("Bottom color", colors[0].name());
    }

    void printNumericalTelemetry(MultipleTelemetry telemetry) {
        printHSV(telemetry, topHSV, "Top HSV");
        telemetry.addLine();
        printHSV(telemetry, bottomHSV, "Bottom HSV");
    }
}
