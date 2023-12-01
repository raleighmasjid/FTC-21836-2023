package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.Direction.REVERSE;
import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1150;
import static org.firstinspires.ftc.teamcode.control.placementalg.ScoringMeasurements.*;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.LowPassGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;

public final class Lift {

    public static PIDGains pidGains = new PIDGains(
            0,
            0,
            0,
            1
    );

    public static LowPassGains filterGains = new LowPassGains(
            0,
            2
    );

    public static double
            kG = 0,
            INCHES_PER_TICK = 1;

    private final HardwareMap hardwareMap;

    // Motors and variables to manage their readings:
    private final MotorEx[] motors;
    private State currentState = new State(), targetState = new State();
    private int targetRow = -1;
    private final FIRLowPassFilter kDFilter = new FIRLowPassFilter(filterGains);
    private final PIDController controller = new PIDController(kDFilter);

    // Battery voltage sensor and variable to track its readings:
    private final VoltageSensor batteryVoltageSensor;
    private double batteryVoltage;

    private MotorEx getLiftMotor(String name) {
        return new MotorEx(hardwareMap, name, RPM_1150);
    }

    public Lift(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.motors = new MotorEx[]{
                getLiftMotor("lift right"),
                getLiftMotor("lift left")
        };
        motors[1].setInverted(true);
        motors[1].encoder.setDirection(REVERSE);
        reset();
    }

    public void readSensors() {
        batteryVoltage = batteryVoltageSensor.getVoltage();
        currentState = new State(INCHES_PER_TICK * (motors[0].encoder.getPosition() + motors[1].encoder.getPosition()) / 2.0);
        kDFilter.setGains(filterGains);
        controller.setGains(pidGains);
    }

    public void incrementRow() {
        setTargetRow(targetRow + 1);
    }

    public void decrementRow() {
        setTargetRow(targetRow - 1);
    }

    public void retract() {
        setTargetRow(-1);
    }

    public void setTargetRow(int targetRow) {
        this.targetRow = max(min(targetRow, 10), -1);
        targetState = new State(this.targetRow < 0 ? 0 : (this.targetRow * PIXEL_HEIGHT + BOTTOM_ROW_HEIGHT));
        controller.setTarget(targetState);
    }

    public int getTargetRow() {
        return targetRow;
    }

    public void run() {
        run(controller.calculate(currentState), false);
    }

    public void run(double output) {
        run(output, true);
    }

    private void run(double output, boolean voltageCompensate) {
        double scalar = 12.0 / batteryVoltage;
        if (voltageCompensate) output *= scalar;
        for (MotorEx motor : motors) motor.set(output + kG() * scalar);
    }

    private double kG() {
        return currentState.x > 0.15 ? kG : 0;
    }

    public void reset() {
        targetRow = -1;
        currentState = new State();
        targetState = new State();
        controller.reset();
        for (MotorEx motor : motors) motor.encoder.reset();
    }

    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Named target position", targetRow < 0 ? "Retracted" : "Row " + targetRow);
    }

    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Current position (in)", currentState.x);
        telemetry.addData("Target position (in)", targetState.x);
        telemetry.addLine();
        telemetry.addData("Lift error derivative (in/s)", controller.getErrorDerivative());

    }
}
