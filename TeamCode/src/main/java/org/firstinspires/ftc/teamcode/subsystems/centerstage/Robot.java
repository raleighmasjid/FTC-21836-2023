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
    public final Intake intake;
    public final Deposit deposit;

    private final List<LynxModule> revHubs;

    public Robot(HardwareMap hardwareMap) {
        revHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : revHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        drivetrain = new AutoTurnMecanum(hardwareMap);
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);
    }

    public void readSensors() {
        for (LynxModule hub : revHubs) hub.clearBulkCache();
    }

    public void run() {

        if (intake.pixelsTransferred()) deposit.paintbrush.lockPixels(intake.getColors());

        deposit.run();
        intake.run(deposit.paintbrush.getPixelsLocked(), deposit.paintbrush.isExtended());
    }

    public void interrupt() {
        drivetrain.interrupt();
        intake.interrupt();
    }

    public void printTelemetry(MultipleTelemetry mTelemetry) {
        drivetrain.printTelemetry(mTelemetry);
        mTelemetry.addLine();
        deposit.paintbrush.printTelemetry(mTelemetry);
        mTelemetry.addLine();
        deposit.lift.printTelemetry(mTelemetry);
        mTelemetry.addLine();
        intake.printTelemetry(mTelemetry);
        mTelemetry.addLine();
        mTelemetry.addLine();
        mTelemetry.addLine();
        drivetrain.printNumericalTelemetry(mTelemetry);
        mTelemetry.addLine();
        deposit.lift.printNumericalTelemetry(mTelemetry);
        mTelemetry.addLine();
        intake.printNumericalTelemetry(mTelemetry);
    }
}
