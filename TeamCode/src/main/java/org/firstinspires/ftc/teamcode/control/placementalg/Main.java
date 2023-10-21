package org.firstinspires.ftc.teamcode.control.placementalg;

import java.util.Scanner;

public class Main {
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
                "_ _ _ _ _ _ _",
                "  _ _ _ _ _ _",
                "W _ _ _ _ _ _",
                "  W _ _ _ _ _",
                "W W _ _ _ _ _",
                "  W W _ _ _ _",
        };

        for (int y = 0; y < colors.length; y++) {
            if (y % 2 == 0) colors[y] = "." + colors[y].substring(1, 13);
            String[] rowList = colors[y].split(" ");
            for (int x = 0; x < rowList.length; x++) {
                if (y % 2 == 0 && x == 0) continue;
                backdrop.add(new Pixel(x, 10 - y, Pixel.Color.fromString(rowList[x])));
            }
        }
        backdrop.calculate();
        backdrop.print();
        printLine();

        boolean solve = false;
        while (backdrop.rowNotFull(10)) {
            if (!solve) {
                int x = input.nextInt();
                int y = input.nextInt();
                input.nextLine();
                String color = input.nextLine();
                if (color.equalsIgnoreCase("solve")) solve = true;
                backdrop.add(new Pixel(x, y, Pixel.Color.fromString(color)));
            }
            if (solve) {
                Pixel pToPlace = backdrop.pixelsToPlace.get(0);
                if (alwaysPlaceColored && pToPlace.color == Pixel.Color.ANY) pToPlace = new Pixel(pToPlace, Pixel.Color.COLORED);
                backdrop.add(pToPlace);
            }
            backdrop.calculate();
            if (!solve || printPerIteration) {
                printLine();
                printLine();
                backdrop.print();
            }
        }
        if (solve && !printPerIteration) {
            printLine();
            printLine();
            backdrop.print();
        }
        System.out.println(backdrop.numOfMosaics + " total mosaics");
    }

    public static void printLine() {
        System.out.println();
    }
}
