package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import java.util.ArrayList;

public final class BackdropScanner extends Thread {

    private boolean newScanAvailable = false, run = true;

    private Backdrop newScan = new Backdrop();

    private ArrayList<Pixel> optimalPlacements = new ArrayList<>();

    public BackdropScanner() {
        start();
    }

    public void run() {
        while (run) {
            Backdrop lastScan = newScan;
            newScan = new Backdrop();

            // Detect (one of three) april tags on the (alliance-specific) backdrop (specified during pre-match config)

            // Skew image to fit april tag to square proportions

            // Shift image to some "standard configuration"

            // Check specific screen pixel coordinates to get colors

            // Save colors to corresponding locations in newScan

            if (!newScan.equals(lastScan)) {
                newScanAvailable = true;
                optimalPlacements = PlacementCalculator.calculate(newScan);
            }
        }
    }

    public boolean newScanAvailable() {
        return newScanAvailable;
    }

    public Backdrop getBackdrop() {
        return newScan;
    }

    public ArrayList<Pixel> getOptimalPlacements() {
        return optimalPlacements;
    }

    public void interrupt() {
        run = false;
    }
}
