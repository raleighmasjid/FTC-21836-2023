package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.drivetrains.AutoTurnMecanum;

import java.util.List;

@Config
public final class Robot {

    public static double maxVoltage = 13;

    public boolean isRed = true;

    public final AutoTurnMecanum drivetrain;
    public final Lift lift;
    public final Intake intake;
    public final Deposit deposit;

    private final List<LynxModule> revHubs;

    public Robot(HardwareMap hardwareMap) {
        revHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : revHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        drivetrain = new AutoTurnMecanum(hardwareMap);
        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);
    }

    public void readSensors() {
        for (LynxModule hub : revHubs) hub.clearBulkCache();
    }

    public void run() {

        if (intake.justDroppedPixels()) deposit.lockPixels(intake.getRequiredPixelCount());
        if (deposit.droppedBothPixels()) lift.retract();
        boolean liftExtended = lift.isExtended();
        if (deposit.isExtended() != liftExtended) deposit.setExtended(liftExtended);

        lift.run();
        intake.run();
        deposit.run();
    }

    public void interrupt() {
        drivetrain.interrupt();
        intake.interrupt();
    }

    public void printTelemetry(MultipleTelemetry telemetry) {
        drivetrain.printTelemetry(telemetry);
        telemetry.addLine();
        lift.printTelemetry(telemetry);
        telemetry.addLine();
        intake.printTelemetry(telemetry);
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        drivetrain.printNumericalTelemetry(telemetry);
        telemetry.addLine();
        lift.printNumericalTelemetry(telemetry);
        telemetry.addLine();
        intake.printNumericalTelemetry(telemetry);
    }
}
