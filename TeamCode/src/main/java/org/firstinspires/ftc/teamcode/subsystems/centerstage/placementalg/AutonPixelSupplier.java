package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.PlacementCalculator;

import java.util.ArrayList;

public final class AutonPixelSupplier {

    public static int getOtherPlacement(int placement) {
        switch (placement) {
            case 1: return 2;
            case 2: return 1;
            case 3: return 4;
            case 4: return 3;
            case 5: return 6;
            default: case 6: return 5;
        }
    }

    /**
     * Calculates the optimal placements for autonomous scoring based on a randomization
     * @param ourPlacement The scoring X that will be considered for the yellow pixel placement and avoidance thereof
     * @param partnerWillDoRandomization Whether or not the alliance partner will place a yellow pixel on the backdrop
     * @return An {@link ArrayList< Pixel >} of optimal placements for autonomous
     */
    public static ArrayList<Pixel> getPlacements(boolean partnerWillDoRandomization, int ourPlacement) {
        int[] yellowPixelXs = partnerWillDoRandomization ?
                new int[]{getOtherPlacement(ourPlacement), ourPlacement} :
                new int[]{ourPlacement};

        PlacementCalculator.auton = true;

        ArrayList<Pixel> placements = new ArrayList<>();

        Backdrop backdrop = new Backdrop();
        for (int x : yellowPixelXs) {
            Pixel yellowEnforced = new Pixel(x, 0, Pixel.Color.YELLOW);
            backdrop.add(yellowEnforced);
            placements.add(yellowEnforced);
        }

        ArrayList<Pixel> optimalPlacements = PlacementCalculator.getOptimalPlacements(backdrop);
        while (backdrop.notFull()) {
            Pixel optimalPlacement = null;
            for (Pixel placement : optimalPlacements) {
                if (placement.color == Pixel.Color.WHITE) {
                    optimalPlacement = placement;
                    break;
                }
            }
            if (optimalPlacement == null) break;
            optimalPlacements = PlacementCalculator.getOptimalPlacements(backdrop.add(optimalPlacement));
            placements.add(optimalPlacement);
        }

        PlacementCalculator.auton = false;

        return placements;
    }
}
