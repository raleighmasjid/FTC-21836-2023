package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.YELLOW;

import java.util.ArrayList;

public final class PlacementGenerator {

    public static void main(String[] args) {

        for (int i = 1; i <= 6; i++) {
            ArrayList<Pixel> optimalPlacements = AutonPixelSupplier.getPlacementsForRandomization(
                    new Pixel(i, 0, YELLOW)
            );
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
