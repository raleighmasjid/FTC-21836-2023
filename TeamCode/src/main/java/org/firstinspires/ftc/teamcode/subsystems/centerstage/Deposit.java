package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1150;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_SECOND;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.maxVoltage;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getAxonServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.FeedforwardGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.KalmanGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.LowPassGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.ProfileConstraints;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;

@Config
public final class Deposit {

    public final Paintbrush paintbrush;
    public final Lift lift;

    Deposit(HardwareMap hardwareMap) {
        lift = new Lift(hardwareMap);
        paintbrush = new Paintbrush(hardwareMap);
    }

    void run() {

        if ((!paintbrush.droppedPixel) && (paintbrush.timer.seconds() >= TIME_DROP_SECOND)) {
            paintbrush.droppedPixel = true;
            lift.setTargetRow(-1);
        }

        boolean liftExtended = lift.targetRow > -1;
        if (paintbrush.pivot.isActivated() != liftExtended) paintbrush.pivot.setActivated(liftExtended);

        lift.run();

        paintbrush.run();
    }

    boolean isRetracted() {
        return !paintbrush.pivot.isActivated() && lift.currentState.x <= 0.5;
    }

    @Config
    public static final class Lift {

        public static PIDGains pidGains = new PIDGains(
                0.05,
                0.025,
                0,
                1
        );

        private double lastKp = pidGains.kP;

        public static FeedforwardGains feedforwardGains = new FeedforwardGains(
                0.01,
                0.01
        );

        public static LowPassGains lowPassGains = new LowPassGains(
                0,
                2
        );

        public static ProfileConstraints profileConstraints = new ProfileConstraints(
                20,
                10,
                5
        );

        public static KalmanGains kalmanGains = new KalmanGains(
                0.3,
                3,
                3
        );

        public static double
                kG = 0.1,
                INCHES_PER_TICK = 0.0322835,
                PERCENT_OVERSHOOT = 0,
                POS_1 = 0,
                POS_2 = 25;

        // Motors and variables to manage their readings:
        private final MotorEx[] motors;
        private final State currentState = new State();
        private final State targetState = new State();
        private int targetRow = -1;
        private final FIRLowPassFilter kDFilter = new FIRLowPassFilter(lowPassGains);
        private final PIDController controller = new PIDController(kDFilter);

        // Battery voltage sensor and variable to track its readings:
        private final VoltageSensor batteryVoltageSensor;

        private Lift(HardwareMap hardwareMap) {
            this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
            this.motors = new MotorEx[]{
                    new MotorEx(hardwareMap, "lift right", RPM_1150),
                    new MotorEx(hardwareMap, "lift left", RPM_1150)
            };
            motors[1].setInverted(true);
            for (MotorEx motor : motors) {
                motor.setZeroPowerBehavior(FLOAT);
                motor.encoder.reset();
            }
        }

        public void setTargetRow(int targetRow) {
            this.targetRow = clip(targetRow, -1, 10);
            targetState.x = this.targetRow == -1 ? 0 : (this.targetRow * Pixel.HEIGHT + Backdrop.BOTTOM_ROW_HEIGHT);
            controller.setTarget(targetState);
        }

        public void changeRow(int deltaRow) {
            setTargetRow(targetRow + deltaRow);
        }

        private final ElapsedTime encoderReadTimer = new ElapsedTime();

        private void run() {
            encoderReadTimer.reset();
            int pos1 = motors[0].encoder.getPosition();
            mTelemetry.addData("pos1 time", encoderReadTimer.seconds());
            encoderReadTimer.reset();
            int pos2 = motors[1].encoder.getPosition();
            mTelemetry.addData("pos2 time", encoderReadTimer.seconds());
            currentState.x = INCHES_PER_TICK * 0.5 * (pos1 + pos2);

            if (lastKp != pidGains.kP) {
                pidGains.computeKd(feedforwardGains, PERCENT_OVERSHOOT);
                lastKp = pidGains.kP;
            }

            kDFilter.setGains(lowPassGains);
            controller.setGains(pidGains);

            double output = controller.calculate(currentState) + kG() * (maxVoltage / batteryVoltageSensor.getVoltage());
            for (MotorEx motor : motors) motor.set(output);
        }

        private double kG() {
            return (currentState.x > 0.15) ? kG : 0;
        }

        void printTelemetry() {
            mTelemetry.addData("Named target position", targetRow < 0 ? "Retracted" : "Row " + targetRow);
        }

        void printNumericalTelemetry() {
            mTelemetry.addData("Current position (in)", currentState.x);
            mTelemetry.addData("Target position (in)", targetState.x);
            mTelemetry.addLine();
            mTelemetry.addData("Lift error derivative (in/s)", controller.getFilteredErrorDerivative());
            mTelemetry.addLine();
            mTelemetry.addData("kD (computed)", pidGains.kD);
        }
    }

    @Config
    public static final class Paintbrush {

        public static double
                ANGLE_PIVOT_OFFSET = 3,
                ANGLE_CLAW_OPEN = 20,
                ANGLE_CLAW_CLOSED = 50,
                ANGLE_HOOK_OPEN = 8,
                ANGLE_HOOK_CLOSED = 40,
                TIME_DROP_FIRST = 1,
                TIME_DROP_SECOND = 1;

        private final SimpleServoPivot pivot, hook, claw;

        private final ElapsedTime timer = new ElapsedTime();
        private boolean droppedPixel = true;
        private int pixelsLocked = 0;
        private final Pixel.Color[] colors = {EMPTY, EMPTY};

        private Paintbrush(HardwareMap hardwareMap) {
            pivot = new SimpleServoPivot(
                    ANGLE_PIVOT_OFFSET,
                    ANGLE_PIVOT_OFFSET + 120,
                    getAxonServo(hardwareMap, "deposit left"),
                    getReversedServo(getAxonServo(hardwareMap, "deposit right"))
            );

            hook = new SimpleServoPivot(
                    ANGLE_HOOK_OPEN,
                    ANGLE_HOOK_CLOSED,
                    getGoBildaServo(hardwareMap, "pixel hook")
            );

            claw = new SimpleServoPivot(
                    ANGLE_CLAW_OPEN,
                    ANGLE_CLAW_CLOSED,
                    getReversedServo(getGoBildaServo(hardwareMap, "pixel claw"))
            );
        }

        int getPixelsLocked() {
            return pixelsLocked;
        }

        Pixel.Color[] getColors() {
            return colors;
        }

        void lockPixels(Pixel.Color[] colors) {
            int pixelsInIntake = 0;
            for (Pixel.Color color : colors) if (color != EMPTY) pixelsInIntake++;

            if (pixelsInIntake >= 1) this.colors[1] = colors[1];
            if (pixelsInIntake == 2) this.colors[0] = colors[0];

            pixelsLocked = clip(pixelsLocked + pixelsInIntake, 0, 2);
        }

        public void dropPixels(int numToDrop) {
            pixelsLocked = clip(pixelsLocked - numToDrop, 0, 2);
            if (pixelsLocked <= 1) colors[0] = EMPTY;
            if (pixelsLocked == 0) {
                colors[1] = EMPTY;
                droppedPixel = false;
                timer.reset();
            }
        }

        private void run() {
            pivot.updateAngles(ANGLE_PIVOT_OFFSET, ANGLE_PIVOT_OFFSET + 120);
            claw.updateAngles(ANGLE_CLAW_OPEN, ANGLE_CLAW_CLOSED);
            hook.updateAngles(ANGLE_HOOK_OPEN, ANGLE_HOOK_CLOSED);

            claw.setActivated(pixelsLocked >= 1);
            hook.setActivated(pixelsLocked == 2);

            pivot.run();
            claw.run();
            hook.run();
        }

        void printTelemetry() {
            mTelemetry.addData("First color", colors[0].name());
            mTelemetry.addData("Second color", colors[1].name());
        }
    }
}
