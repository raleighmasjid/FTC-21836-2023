package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1150;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.maxVoltage;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getAxonServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.LowPassGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;

@Config
public final class Deposit {

    final Paintbrush paintbrush;
    public final Lift lift;

    Deposit(HardwareMap hardwareMap) {
        lift = new Lift(hardwareMap);
        paintbrush = new Paintbrush(hardwareMap);
    }

    public void dropPixels(int numToDrop) {
        paintbrush.dropPixels(numToDrop);
    }

    void run() {

        if (paintbrush.droppedBothPixels()) lift.retract();
        boolean liftExtended = lift.isExtended();
        if (paintbrush.isExtended() != liftExtended) paintbrush.setExtended(liftExtended);

        lift.run();
        paintbrush.run();
    }

    @Config
    public static final class Lift {

        public static PIDGains pidGains = new PIDGains(
                0.05,
                0.025,
                0,
                1
        );

        public static LowPassGains filterGains = new LowPassGains(
                0.8,
                10
        );

        public static double
                kG = 0,
                INCHES_PER_TICK = 0.0322835;

        // Motors and variables to manage their readings:
        private final MotorEx[] motors;
        private State currentState = new State(), targetState = new State();
        private int targetRow = -1;
        private final FIRLowPassFilter kDFilter = new FIRLowPassFilter(filterGains);
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
            for (MotorEx motor : motors) motor.setZeroPowerBehavior(FLOAT);
            reset();
        }

        public void setTargetRow(int targetRow) {
            this.targetRow = clip(targetRow, -1, 10);
            targetState = new State(this.targetRow == -1 ? 0 : (this.targetRow * Pixel.HEIGHT + Backdrop.BOTTOM_ROW_HEIGHT));
            controller.setTarget(targetState);
        }

        public void changeRow(int deltaRow) {
            setTargetRow(targetRow + deltaRow);
        }

        private void retract() {
            setTargetRow(-1);
        }

        private boolean isExtended() {
            return targetRow > -1;
        }

        private void run() {

            currentState = new State(INCHES_PER_TICK * (motors[0].encoder.getPosition() + motors[1].encoder.getPosition()) / 2.0);

            kDFilter.setGains(filterGains);
            controller.setGains(pidGains);

            double output = controller.calculate(currentState) + kG() * (maxVoltage / batteryVoltageSensor.getVoltage());
            for (MotorEx motor : motors) motor.set(output);
        }

        private double kG() {
            return currentState.x > 0.15 ? kG : 0;
        }

        private void reset() {
            targetRow = -1;
            currentState = new State();
            targetState = new State();
            controller.reset();
            for (MotorEx motor : motors) motor.encoder.reset();
        }

        void printTelemetry(MultipleTelemetry telemetry) {
            telemetry.addData("Named target position", targetRow < 0 ? "Retracted" : "Row " + targetRow);
        }

        void printNumericalTelemetry(MultipleTelemetry telemetry) {
            telemetry.addData("Current position (in)", currentState.x);
            telemetry.addData("Target position (in)", targetState.x);
            telemetry.addLine();
            telemetry.addData("Lift error derivative (in/s)", controller.getErrorDerivative());

        }
    }

    @Config
    static final class Paintbrush {

        public static double
                ANGLE_PIVOT_OFFSET = 5,
                ANGLE_CLAW_OPEN = 0,
                ANGLE_CLAW_CLOSED = 15,
                ANGLE_HOOK_OPEN = 5,
                ANGLE_HOOK_CLOSED = 40,
                TIME_DROP = 1;

        private final SimpleServoPivot pivot, hook, claw;

        private final ElapsedTime timer = new ElapsedTime();
        private boolean retracted = true;
        private int pixelsLocked = 0;

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

        void lockPixels(Pixel.Color[] colors) {
            int pixelCount = 0;
            for (Pixel.Color color : colors) if (color != EMPTY) pixelCount++;
            pixelsLocked = clip(pixelCount, 0, 2);
        }

        private void dropPixels(int numToDrop) {
            pixelsLocked = clip(pixelsLocked - numToDrop, 0, 2);
            if (pixelsLocked == 0) {
                retracted = false;
                timer.reset();
            }
        }

        private void setExtended(boolean extended) {
            pivot.setActivated(extended);
        }

        private boolean isExtended() {
            return pivot.getActivated();
        }

        private boolean droppedBothPixels() {
            return !retracted && timer.seconds() >= TIME_DROP;
        }

        private void run() {
            if (droppedBothPixels()) retracted = true;

            pivot.updateAngles(ANGLE_PIVOT_OFFSET, ANGLE_PIVOT_OFFSET + 120);
            claw.updateAngles(ANGLE_CLAW_OPEN, ANGLE_CLAW_CLOSED);
            hook.updateAngles(ANGLE_HOOK_OPEN, ANGLE_HOOK_CLOSED);

            claw.setActivated(pixelsLocked >= 1);
            hook.setActivated(pixelsLocked == 2);

            pivot.run();
            claw.run();
            hook.run();
        }
    }
}
