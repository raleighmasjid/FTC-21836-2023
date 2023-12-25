package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.ANY;
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
    }

    public static ArrayList<Pixel> getPlacements(
            Randomization randomization,
            boolean partnerWillDoRandomization
    ) {
        if (partnerWillDoRandomization) {
            return getPlacementsForRandomization(
                    new Pixel(randomization.x1, 0, YELLOW),
                    new Pixel(randomization.x2, 0, YELLOW)
            );
        }
        return getPlacementsForRandomization(new Pixel(randomization.x1, 0, YELLOW));
    }

    public static ArrayList<Pixel> getPlacementsForRandomization(Pixel... yellowPixels) {
        ArrayList<Pixel> placements = new ArrayList<>();

        Backdrop backdrop = new Backdrop();
        for (Pixel pixel : yellowPixels) {
            Pixel yellowEnforced = new Pixel(pixel, YELLOW);
            backdrop.add(yellowEnforced);
            placements.add(yellowEnforced);
        }
        ArrayList<Pixel> optimalPlacements = getOptimalPlacements(backdrop);

        while (backdrop.notFull()) {
            Pixel optimalPlacement = null;
            for (Pixel placement : optimalPlacements) {
                if (placement.color == ANY || placement.color == WHITE) {
                    optimalPlacement = new Pixel(placement, WHITE);
                    break;
                }
            }
            if (optimalPlacement == null) break;
            optimalPlacements = getOptimalPlacements(backdrop.add(optimalPlacement));
            placements.add(optimalPlacement);
        }

        return placements;
    }
}
