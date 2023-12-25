package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.AutonPixelSupplier.Randomization.CENTER;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.AutonPixelSupplier.Randomization.LEFT;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.AutonPixelSupplier.Randomization.RIGHT;

import java.util.ArrayList;

public final class AutonPlacementVisualizer {

    public static void main(String[] args) {

        for (AutonPixelSupplier.Randomization rand : new AutonPixelSupplier.Randomization[]{LEFT, CENTER, RIGHT}) {
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

/*
1 : 30 : 6
2 : 29 :
3 : 17 :
4 : 17 :
5 : 29 :
6 : 30 : 6
 */
