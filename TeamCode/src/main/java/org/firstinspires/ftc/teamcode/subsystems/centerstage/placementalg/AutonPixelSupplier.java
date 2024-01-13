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

    /**
     * Calculates the optimal placements for autonomous scoring based on a randomization
     * @param randomization The randomization that will be considered for the yellow pixel placement and avoidance thereof
     * @param partnerWillDoRandomization Whether or not the alliance partner will place a yellow pixel on the backdrop
     * @return An {@link ArrayList<Pixel>} of optimal placements for autonomous
     */
    public static ArrayList<Pixel> getPlacements(Randomization randomization, boolean partnerWillDoRandomization) {
        return partnerWillDoRandomization ?
                getWhitePixelPlacements(randomization.x1, randomization.x2) :
                getWhitePixelPlacements(randomization.x1);
    }

    /**
     * Calculates the optimal placements of white pixels during autonomous to achieve setline bonuses upon teleop start
     * @param yellowPixelXs The x values on the backdrop where yellow pixels will be placed as part of randomization
     * @return An {@link ArrayList<Pixel>} of optimal placements for autonomous
     */
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
