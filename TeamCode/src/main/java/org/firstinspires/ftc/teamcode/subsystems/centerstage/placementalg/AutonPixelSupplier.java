package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.WHITE;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.YELLOW;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.PlacementCalculator.getOptimalPlacements;

import java.util.ArrayList;

public final class AutonPixelSupplier {

    public enum Randomization {
        LEFT(1, 2),
        CENTER(3, 4),
        RIGHT(6, 5);

        Randomization(int x1, int x2) {
            this.x1 = x1;
            this.x2 = x2;
        }

        final int x1, x2;
        static final Randomization[] randomizations = values();
    }

    public static ArrayList<Pixel> getPlacements(Randomization randomization, boolean partnerWillDoRandomization) {
        return partnerWillDoRandomization ?
                getWhitePixelPlacements(randomization.x1, randomization.x2) :
                getWhitePixelPlacements(randomization.x1);
    }

    private static ArrayList<Pixel> getWhitePixelPlacements(int... yellowPixelXs) {
        PlacementCalculator.auton = true;

        ArrayList<Pixel> placements = new ArrayList<>();

        Backdrop backdrop = new Backdrop();
        for (int x : yellowPixelXs) {
            Pixel yellowEnforced = new Pixel(x, 0, YELLOW);
            backdrop.add(yellowEnforced);
            placements.add(yellowEnforced);
        }

        ArrayList<Pixel> optimalPlacements = getOptimalPlacements(backdrop);
        while (backdrop.notFull()) {
            Pixel optimalPlacement = null;
            for (Pixel placement : optimalPlacements) {
                if (placement.color == WHITE) {
                    optimalPlacement = placement;
                    break;
                }
            }
            if (optimalPlacement == null) break;
            optimalPlacements = getOptimalPlacements(backdrop.add(optimalPlacement));
            placements.add(optimalPlacement);
        }

        PlacementCalculator.auton = false;

        return placements;
    }
}
