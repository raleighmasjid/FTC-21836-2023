package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.PlacementCalculator;

import java.util.ArrayList;

public final class AutonPixelSupplier {

    private static final PlacementCalculator calculator = new PlacementCalculator(true);

    public static int getOtherPlacement(int i) {
        return i + (i % 2 == 0 ? -1 : 1);
    }

    /**
     * Calculates the optimal placements for autonomous scoring based on a randomization
     * @param ourPlacement The scoring X that will be considered for the yellow pixel placement and avoidance thereof
     * @param partnerWillDoRandomization Whether or not the alliance partner will place a yellow pixel on the backdrop
     * @return An {@link ArrayList<Pixel>} of optimal placements for autonomous
     */
    public static ArrayList<Pixel> getPlacements(boolean partnerWillDoRandomization, int ourPlacement) {
        int[] yellowPixelXs = partnerWillDoRandomization ?
                new int[]{getOtherPlacement(ourPlacement), ourPlacement} :
                new int[]{ourPlacement};

        ArrayList<Pixel> placements = new ArrayList<>();

        Backdrop backdrop = new Backdrop();
        for (int x : yellowPixelXs) {
            Pixel yellowEnforced = new Pixel(x, 0, Pixel.Color.YELLOW);
            backdrop.add(yellowEnforced);
            placements.add(yellowEnforced);
        }

        ArrayList<Pixel> optimalPlacements = calculator.getOptimalPlacements(backdrop);
        while (backdrop.notFull()) {
            Pixel optimalPlacement = null;
            for (Pixel placement : optimalPlacements) {
                if (placement.color != Pixel.Color.WHITE) continue;
                optimalPlacement = placement;
                break;
            }
            if (optimalPlacement == null) break;
            optimalPlacements = calculator.getOptimalPlacements(backdrop.add(optimalPlacement));
            placements.add(optimalPlacement);
        }

        return placements;
    }
}
