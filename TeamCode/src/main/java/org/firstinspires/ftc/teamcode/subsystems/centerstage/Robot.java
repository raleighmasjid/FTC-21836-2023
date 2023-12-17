package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.BackdropScanner;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.PlacementCalculator;
import org.firstinspires.ftc.teamcode.subsystems.drivetrains.AutoTurnMecanum;

import java.util.ArrayList;
import java.util.List;

@Config
public final class Robot {

    public static double maxVoltage = 13;

    public boolean isRed = true;

    public final AutoTurnMecanum drivetrain;
    public final Intake intake;
    public final Deposit deposit;

    private final List<LynxModule> revHubs;
    private Backdrop latestScan = null;
    private final BackdropScanner scanner = new BackdropScanner();
    private final Pixel[] placements = new Pixel[2];

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

        scanner.run();
        if (scanner.newScanAvailable()) latestScan = scanner.getBackdrop();

        if (intake.pixelsTransferred()) {
            deposit.paintbrush.lockPixels(intake.getColors());

            if (latestScan != null) {
                placements[0] = null;
                placements[1] = null;

                ArrayList<Pixel> optPlacements = PlacementCalculator.calculate(latestScan);
                Pixel.Color[] depositColors = deposit.paintbrush.getColors();
                Pixel.Color firstColor = depositColors[0], secondColor = depositColors[1];

                if (firstColor != EMPTY) for (Pixel pixel1 : optPlacements) {
                    if (firstColor.matches(pixel1.color)) {

                        Pixel placement = new Pixel(pixel1, firstColor);

                        placements[0] = placement;
                        latestScan.add(placement);
                        optPlacements = PlacementCalculator.calculate(latestScan);
                        break;
                    }
                }
                if (secondColor != EMPTY) for (Pixel pixel : optPlacements) {
                    if (secondColor.matches(pixel.color)) {

                        Pixel placement = new Pixel(pixel, secondColor);

                        placements[1] = placement;
                        latestScan.add(placement);
                        break;
                    }
                }
            }
        }

        deposit.run();
        intake.run(deposit.paintbrush.getPixelsLocked(), deposit.isRetracted());
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
