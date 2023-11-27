package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.drivetrains.AutoTurnMecanum;

public class Robot {

    public AutoTurnMecanum drivetrain;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new AutoTurnMecanum(hardwareMap);
    }

    public void readSensors() {
        drivetrain.updateGains();
    }

    public void start() {
        drivetrain.imu.start();
    }

    public void interrupt() {
        drivetrain.imu.interrupt();
    }

    public void printTelemetry(MultipleTelemetry telemetry) {
        drivetrain.printTelemetry(telemetry);
    }

    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        drivetrain.printNumericalTelemetry(telemetry);
    }
}
