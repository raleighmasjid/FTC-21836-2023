package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.placementalg.PlacementCalculator;
import org.firstinspires.ftc.teamcode.subsystems.drivetrains.AutoTurnMecanum;

public class Robot {

    public AutoTurnMecanum drivetrain;
    public Intake intake;
    public Deposit deposit;
    public Lift lift;
    private PlacementCalculator calculator;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new AutoTurnMecanum(hardwareMap);
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);
        lift = new Lift(hardwareMap);
    }

    public void readSensors() {
        drivetrain.updateGains();
        lift.readSensors();
    }

    public void start() {
        drivetrain.imu.start();
        intake.start();
    }

    public void interrupt() {
        drivetrain.imu.interrupt();
        intake.interrupt();
    }

    public void printTelemetry(MultipleTelemetry telemetry) {
        drivetrain.printTelemetry(telemetry);
        telemetry.addLine();
        lift.printTelemetry(telemetry);
    }

    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        drivetrain.printNumericalTelemetry(telemetry);
        telemetry.addLine();
        lift.printNumericalTelemetry(telemetry);
        telemetry.addLine();
        intake.printNumericalTelemetry(telemetry);
    }
}
