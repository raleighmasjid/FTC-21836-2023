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
                "  _ _ _ _ _ _",
                "_ _ _ _ _ _ _",
                "  _ _ _ _ _ _",
                "_ _ _ _ _ _ _",
                "  _ _ _ _ _ _",
                "W W G _ _ _ _",
                "  p W W _ _ _",
                "g y W g _ _ _",
                "  W W y p _ _",
                "g p W W W _ _",
                "  y W P Y W _",
        };

        for (int y = 0; y < colors.length; y++) {
            if (y % 2 == 0) colors[y] = "." + colors[y].substring(1, 13);
            String[] rowList = colors[y].split(" ");
            for (int x = 0; x < rowList.length; x++) {
                if (y % 2 == 0 && x == 0) continue;
                backdrop.add(new Pixel(x, 10 - y, Pixel.Color.fromString(rowList[x])));
            }
        }
        ArrayList<Pixel> pixelsToPlace = PlacementCalculator.calculate(backdrop);
        backdrop.print();
        printLine();
        for (Pixel pixel : pixelsToPlace) pixel.print();
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
                Pixel pToPlace = pixelsToPlace.get(0);
                if (alwaysPlaceColored && pToPlace.color == ANY) pToPlace = new Pixel(pToPlace, COLORED);
                backdrop.add(pToPlace);
            }
            pixelsToPlace = PlacementCalculator.calculate(backdrop);
            if (!solve || printPerIteration) {
                printLine();
                printLine();
                backdrop.print();
                printLine();
                for (Pixel pixel : pixelsToPlace) pixel.print();
            }
        }
        if (solve && !printPerIteration) {
            printLine();
            printLine();
            backdrop.print();
            printLine();
            for (Pixel pixel : pixelsToPlace) pixel.print();
        }
        System.out.println(backdrop.mosaicCount + " mosaics");
    }

    private static void printLine() {
        System.out.println();
    }
}
