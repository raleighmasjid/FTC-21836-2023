package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel;

import java.util.ArrayList;

public final class AutonPlacementVisualizer {

    public static void main(String[] args) {

        for (int i : new int[]{1, 3, 6}) {
            for (boolean partnerWillDoRandomization : new boolean[]{true, false}) {
                ArrayList<Pixel> optimalPlacements = AutonPixelSupplier.getPlacements(partnerWillDoRandomization, i);
                Backdrop backdrop = new Backdrop();

                for (int j = 0; j < 15; j++) {
                    backdrop.add(optimalPlacements.get(j));
                }
                backdrop.print();
            }
        }
    }

    static class Bah {
        public static void main(String... args) {
//            Backdrop backdrop = new Backdrop();
            for (Pixel placement : AutonPixelSupplier.getPlacements(false, 3)) {
                System.out.println(placement.x + ", " + placement.y + ", " + placement.color.name());
//                backdrop.add(placement).print();
            }
        }
    }
}
