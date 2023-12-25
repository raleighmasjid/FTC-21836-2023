package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.ANY;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.COLORED;

import java.util.ArrayList;
import java.util.Scanner;

final class AlgorithmTesting {
    public static void main(String[] args) {

        Scanner input = new Scanner(System.in);
        Backdrop backdrop = new Backdrop();
        boolean alwaysPlaceColored = true;
        boolean printPerIteration = true;
        backdrop.printRectangular = false;

        String[] colors = {
                " _ _ _ _ _ _",
                "_ _ _ _ _ _ _",
                " _ _ _ _ _ _",
                "_ _ _ _ _ _ _",
                " _ _ _ _ _ _",
                "W _ G _ _ _ _",
                " p W W _ _ _",
                "g y W g _ _ _",
                " W W y p _ _",
                "g p W W W _ _",
                " y W P Y W _",
        };

        for (int y = 0; y < colors.length; y++) {
            if (y % 2 == 0) colors[y] = "." + colors[y].substring(0, 12);
            String[] rowList = colors[y].split(" ");
            for (int x = 0; x < rowList.length; x++) {
                if (y % 2 == 0 && x == 0) continue;
                backdrop.add(new Pixel(x, 10 - y, Pixel.Color.fromString(rowList[x])));
            }
        }
        ArrayList<Pixel> optimalPlacements = PlacementCalculator.getOptimalPlacements(backdrop);
        backdrop.print();
        printLine();
        for (Pixel pixel : optimalPlacements) pixel.print();
        printLine();

        boolean solve = false;
        while (backdrop.notFull()) {
            if (!solve) {
                int x = input.nextInt();
                int y = input.nextInt();
                input.nextLine();
                String color = input.nextLine();
                if (color.equalsIgnoreCase("solve")) solve = true;
                backdrop.add(new Pixel(x, y, Pixel.Color.fromString(color)));
            }
            if (solve) {
                Pixel placement = optimalPlacements.get(0);
                backdrop.add(alwaysPlaceColored && placement.color == ANY ? new Pixel(placement, COLORED) : placement);
            }
            optimalPlacements = PlacementCalculator.getOptimalPlacements(backdrop);
            if (!solve || printPerIteration) {
                printLine();
                printLine();
                backdrop.print();
                printLine();
                for (Pixel pixel : optimalPlacements) pixel.print();
            }
        }
        if (solve && !printPerIteration) {
            printLine();
            printLine();
            backdrop.print();
            printLine();
            for (Pixel pixel : optimalPlacements) pixel.print();
        }
        System.out.println(backdrop.mosaicCount + " mosaics");
    }

    private static void printLine() {
        System.out.println();
    }
}
