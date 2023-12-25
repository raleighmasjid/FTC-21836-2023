package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.AutonPixelSupplier.Randomization.randomizations;

import java.util.ArrayList;

public final class AutonPlacementVisualizer {

    public static void main(String[] args) {

        for (AutonPixelSupplier.Randomization rand : randomizations) {
            ArrayList<Pixel> optimalPlacements = AutonPixelSupplier.getPlacements(rand, true);
            Backdrop backdrop = new Backdrop();

            for (int j = 0; j < 15; j++) {
                Pixel placement = optimalPlacements.get(j);
                backdrop.add(placement);
            }
            backdrop.print();
        }
    }
}
