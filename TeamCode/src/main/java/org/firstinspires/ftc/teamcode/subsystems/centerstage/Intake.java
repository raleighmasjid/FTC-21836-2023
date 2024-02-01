package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1620;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.HAS_0_PIXELS;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.HAS_1_PIXEL;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.PIVOTING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.PIXELS_FALLING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.PIXELS_SETTLING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.PIXEL_1_SETTLING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.State.PIXEL_2_SETTLING;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.GREEN;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.PURPLE;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.WHITE;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.YELLOW;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getAxonServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;
import static java.lang.Math.asin;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;
import org.firstinspires.ftc.teamcode.subsystems.utilities.sensors.ColorSensor;

@Config
public final class Intake {

    public static double
            ANGLE_PIVOT_OFFSET = 11,
            ANGLE_PIVOT_FLOOR_CLEARANCE = 5,
            ANGLE_PIVOT_TRANSFERRING = 196.1,
            ANGLE_PIVOT_VERTICAL = 50,
            ANGLE_LATCH_INTAKING = 105,
            ANGLE_LATCH_LOCKED = 159,
            ANGLE_LATCH_TRANSFERRING = 0,
            TIME_PIXEL_1_SETTLING = 0.25,
            TIME_PIXEL_2_SETTLING = 0,
            TIME_REVERSING = 1,
            TIME_PIVOTING = 0,
            TIME_SETTLING = 0.2,
            COLOR_SENSOR_GAIN = 1,
            SPEED_SLOW_REVERSING = -0.25,
            HEIGHT_SHIFT = -0.1,
            r = 9.5019488189,
            theta0 = -0.496183876745;

    /**
     * HSV value bound for intake pixel detection
     */
    public static HSV
            minWhite = new HSV(
                    0,
                    0,
                    0.05
            ),
            maxWhite = new HSV(
                    0,
                    0.6,
                    0.45
            ),
            minPurple = new HSV(
                    205,
                    0.55,
                    0.02
            ),
            maxPurple = new HSV(
                    225,
                    1,
                    0.35
            ),
            minYellow = new HSV(
                    90,
                    0.55,
                    0.02
            ),
            maxYellow = new HSV(
                    125,
                    1,
                    0.15
            ),
            minGreen = new HSV(
                    130,
                    0.5,
                    0.01
            ),
            maxGreen = new HSV(
                    160,
                    1,
                    0.2
            );

    private final MotorEx motor;

    final ColorSensor bottomSensor, topSensor;
    private HSV bottomHSV = new HSV(), topHSV = new HSV();

    private final TouchSensor pivotSensor;

    private final SimpleServoPivot pivot, latch;

    private Intake.State state = HAS_0_PIXELS;
    private Intake.Height height = FLOOR;

    private final ElapsedTime timer = new ElapsedTime();
    public final Pixel.Color[] colors = {EMPTY, EMPTY};

    private boolean pixelsTransferred = false, vertical = false;
    private int requiredIntakingAmount = 2;
    private double motorPower = 0;

    enum State {
        HAS_0_PIXELS,
        PIXEL_1_SETTLING,
        PIXEL_2_SETTLING,
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

        public final double deltaX, deltaTheta;

        private static final Intake.Height[] values = values();

        public static Intake.Height get(int ordinal) {
            return values[ordinal];
        }

        public Intake.Height minus(int less) {
            return values[max(ordinal() - less, 0)];
        }

        Height() {
            if (ordinal() == 0) {
                deltaTheta = 0;
                deltaX = 0;
                return;
            }

            double deltaY = ordinal() * 0.5 + HEIGHT_SHIFT;

            double theta1 = asin((r * sin(theta0) + deltaY) / r);
            deltaTheta = toDegrees(theta1 - theta0);
            deltaX = r * cos(theta1) - r * cos(theta0);
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

        bottomSensor = new ColorSensor(hardwareMap, "bottom color", (float) COLOR_SENSOR_GAIN);
        topSensor = new ColorSensor(hardwareMap, "top color", (float) COLOR_SENSOR_GAIN);

        pivotSensor = hardwareMap.get(TouchSensor.class, "intake pivot sensor");

        timer.reset();
    }

    /**
     * @return The {@link Pixel.Color} corresponding to the provided {@link HSV} as per the tuned value bounds
     */
    public static Pixel.Color fromHSV(HSV hsv) {
        return
                hsv.between(minPurple, maxPurple) ? PURPLE :
                hsv.between(minGreen, maxGreen) ? GREEN :
                hsv.between(minYellow, maxYellow) ? YELLOW :
                new HSV(0, hsv.saturation, hsv.value).between(minWhite, maxWhite) ? WHITE :
                EMPTY;
    }

    void run(int pixelsInDeposit, boolean depositRetracted) {

        if (pixelsTransferred) pixelsTransferred = false;

        switch (state) {
            case HAS_0_PIXELS:

                bottomHSV = bottomSensor.getHSV();
                colors[0] = fromHSV(bottomHSV);
                boolean bottomFull = !(colors[0] == EMPTY);
                if (bottomFull || requiredIntakingAmount == 0) {
                    if (bottomFull) decrementHeight();
                    state = PIXEL_1_SETTLING;
                    timer.reset();
                } else break;

            case PIXEL_1_SETTLING:

                if (timer.seconds() >= TIME_PIXEL_1_SETTLING || requiredIntakingAmount == 0)
                    state = HAS_1_PIXEL;
                else break;

            case HAS_1_PIXEL:

                topHSV = topSensor.getHSV();
                colors[1] = fromHSV(topHSV);
                boolean topFull = !(colors[1] == EMPTY);
                if (topFull || requiredIntakingAmount <= 1) {
                    if (topFull) decrementHeight();
                    if (requiredIntakingAmount > 0) latch.setActivated(true);
                    state = PIXEL_2_SETTLING;
                    timer.reset();
                } else break;

            case PIXEL_2_SETTLING:

                if (depositRetracted && (requiredIntakingAmount == 0 || (
                        timer.seconds() >= TIME_PIXEL_2_SETTLING &&
                                requiredIntakingAmount + pixelsInDeposit <= 2
                ))) {
                    state = PIVOTING;
                    pivot.setActivated(true);
                    timer.reset();
                } else break;

            case PIVOTING:

                if (timer.seconds() >= TIME_PIVOTING && pivotSensor.isPressed()) {
                    setMotorPower(0);
                    state = PIXELS_FALLING;
                    latch.setActivated(false);
                } else {
                    setMotorPower(timer.seconds() <= TIME_REVERSING ? -1 : SPEED_SLOW_REVERSING);
                    break;
                }

            case PIXELS_FALLING:

                if (fromHSV(topSensor.getHSV()) == EMPTY &&
                    fromHSV(bottomSensor.getHSV()) == EMPTY
                ) {
                    state = PIXELS_SETTLING;
                    timer.reset();
                } else break;

            case PIXELS_SETTLING:

                if (requiredIntakingAmount > 0 && (pixelsTransferred = timer.seconds() >= TIME_SETTLING)) {
                    state = HAS_0_PIXELS;
                    pivot.setActivated(false);
                } else break;

        }

        pivot.updateAngles(
                ANGLE_PIVOT_OFFSET +
                        (motorPower <= 0 && height == FLOOR ? ANGLE_PIVOT_FLOOR_CLEARANCE : 0) +
                        height.deltaTheta +
                        ((vertical && fromHSV(bottomSensor.getHSV()) == EMPTY) ? ANGLE_PIVOT_VERTICAL : 0),
                ANGLE_PIVOT_OFFSET + ANGLE_PIVOT_TRANSFERRING
        );
        latch.updateAngles(
                state != PIVOTING && state != PIXELS_FALLING && state != PIXELS_SETTLING && requiredIntakingAmount > 0 ? ANGLE_LATCH_INTAKING : ANGLE_LATCH_TRANSFERRING,
                ANGLE_LATCH_LOCKED
        );

        pivot.run();
        latch.run();

        motor.set(motorPower);
    }

    boolean pixelsTransferred() {
        return pixelsTransferred;
    }

    private void decrementHeight() {
        setHeight(height.minus(1));
    }

    public void setHeight(Intake.Height height) {
        this.height = height;
    }

    public void setMotorPower(double motorPower) {
        this.motorPower = motorPower;
    }

    public void setRequiredIntakingAmount(int pixelCount) {
        this.requiredIntakingAmount = clip(pixelCount, 0, 2);
    }

    public void toggleVertical() {
        setVertical(!vertical);
    }

    public void setVertical(boolean vertical) {
        this.vertical = vertical;
    }

    void printTelemetry() {
        mTelemetry.addData("Top color", colors[1].name());
        mTelemetry.addData("Bottom color", colors[0].name());
    }

    void printNumericalTelemetry() {
        topHSV.toTelemetry("Top HSV");
        mTelemetry.addLine();
        bottomHSV.toTelemetry("Bottom HSV");
    }
}
