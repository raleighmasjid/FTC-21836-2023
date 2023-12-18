package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;
import static java.lang.Double.min;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.PlacementCalculator;
import org.firstinspires.ftc.teamcode.subsystems.drivetrains.AutoTurnMecanum;

import java.util.ArrayList;
import java.util.List;

@Config
public final class Robot {

    public static double
            maxVoltage = 13,
            RUNNING_OFFSET_X = 10,
            RUNNING_OFFSET_Y = 10;

    public boolean isRed = true;

    public final AutoTurnMecanum drivetrain;
    public final Intake intake;
    public final Deposit deposit;

    private final List<LynxModule> revHubs;
    private Backdrop latestScan = null;
//    private final BackdropScanner scanner = new BackdropScanner();
    private final Pixel[] placements = {new Pixel((isRed ? -2 : 9), 0, EMPTY), new Pixel((isRed ? -2 : 9), 0, EMPTY)};
    private ArrayList<Pixel> optimalPlacements = new ArrayList<>();

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

//        if (scanner.newScanAvailable()) {
//            latestScan = scanner.getBackdrop();
//            optimalPlacements = scanner.getOptimalPlacements();
//        }

        if (intake.pixelsTransferred()) {
            deposit.paintbrush.lockPixels(intake.getColors());

            if (latestScan != null) {

                Backdrop latestScanCopy = latestScan.clone();
                ArrayList<Pixel> optimalPlacementsCopy = new ArrayList<>(optimalPlacements);

                Pixel.Color[] depositColors = deposit.paintbrush.getColors();
                Pixel.Color firstColor = depositColors[0], secondColor = depositColors[1];

                placements[0] = new Pixel((isRed ? -2 : 9), 0, EMPTY);
                placements[1] = new Pixel((isRed ? -2 : 9), 0, EMPTY);

                if (firstColor != EMPTY) for (Pixel pixel : optimalPlacementsCopy) {
                    if (firstColor.matches(pixel.color)) {

                        Pixel placement = new Pixel(pixel, firstColor);

                        placements[0] = placement;
                        latestScanCopy.add(placement);
                        optimalPlacementsCopy = PlacementCalculator.calculate(latestScanCopy);
                        break;
                    }
                }
                if (secondColor != EMPTY) for (Pixel pixel : optimalPlacementsCopy) {
                    if (secondColor.matches(pixel.color)) {

                        Pixel placement = new Pixel(pixel, secondColor);

                        placements[1] = placement;
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
        for (int i = 0; i < min(2, optimalPlacements.size()); i++) {
            mTelemetry.addLine(optimalPlacements.get(i).color.name());
        }
        mTelemetry.addLine();
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
