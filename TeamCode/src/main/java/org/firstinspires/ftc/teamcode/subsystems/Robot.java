package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.drivetrains.AutoTurnMecanum;

import java.util.List;

public class Robot {

    public boolean isRed = true;

    public final AutoTurnMecanum drivetrain;

    private final List<LynxModule> revHubs;

    public Robot(HardwareMap hardwareMap) {
        revHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : revHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        drivetrain = new AutoTurnMecanum(hardwareMap);
    }

    public void readSensors() {
        for (LynxModule hub : revHubs) hub.clearBulkCache();

        drivetrain.updateGains();
    }

    public void start() {
        drivetrain.start();
    }

    public void interrupt() {
        drivetrain.interrupt();
    }

    public void printTelemetry(MultipleTelemetry telemetry) {
        drivetrain.printTelemetry(telemetry);
        telemetry.addLine();
        telemetry.addLine();
        drivetrain.printNumericalTelemetry(telemetry);
    }
}
