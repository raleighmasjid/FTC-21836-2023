package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.CENTER;
import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.randomizations;

import org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline;

import java.util.ArrayList;

public final class AutonPlacementVisualizer {

    public static void main(String[] args) {

        for (PropDetectPipeline.Randomization rand : randomizations) {
            for (boolean partnerWillDoRandomization : new boolean[]{true, false}) {
                ArrayList<Pixel> optimalPlacements = AutonPixelSupplier.getPlacements(rand, partnerWillDoRandomization);
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
            for (Pixel placement : AutonPixelSupplier.getPlacements(CENTER, false)) {
                System.out.println(placement.x + ", " + placement.y + ", " + placement.color.name());
//                backdrop.add(placement).print();
            }
        }
    }
}
