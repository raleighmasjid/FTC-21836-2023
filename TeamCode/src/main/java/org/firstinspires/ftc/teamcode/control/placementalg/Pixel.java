package org.firstinspires.ftc.teamcode.control.placementalg;

import java.util.ArrayList;
import java.util.Arrays;

public class Pixel implements Comparable<Pixel> {

    public int compareTo(Pixel cPixel) {
        double diff = cPixel.scoreValue - scoreValue;
        if (diff == 0) diff = y - cPixel.y;
        if (diff == 0) diff = x - cPixel.x;
        return (int) (diff * 1000000000);
    }

    public enum Color {
        INVALID,
        EMPTY,
        ANY,
        COLORED,
        WHITE,
        PURPLE,
        GREEN,
        YELLOW;

        private static final String RESET = "\u001B[0m";
        private static final String G = "\u001B[32m";
        private static final String Y = "\u001B[33m";
        private static final String P = "\u001B[35m";

        public String toString() {
            switch (this) {
                case WHITE:
                    return "W";
                case ANY:
                    return "A";
                case COLORED:
                    return "C" + RESET;
                case PURPLE:
                    return P + "P" + RESET;
                case YELLOW:
                    return Y + "Y" + RESET;
                case GREEN:
                    return G + "G" + RESET;
                case INVALID:
                    return " ";
                case EMPTY:
                default:
                    return "_";
            }
        }

        public static Color fromString(String color) {
            switch (color.toUpperCase()) {
                case "W":
                    return Color.WHITE;
                case "A":
                    return Color.ANY;
                case "C":
                    return Color.COLORED;
                case "P":
                    return Color.PURPLE;
                case "Y":
                    return Color.YELLOW;
                case "G":
                    return Color.GREEN;
                case " ":
                    return Color.INVALID;
                case "_":
                default:
                    return Color.EMPTY;
            }
        }

        public static Color getRemainingColor(Color c1, Color c2) {
            if (c1 == Color.EMPTY || c2 == Color.EMPTY) return Color.COLORED;
            if (c1 == c2) return c1;
            ArrayList<Color> colors = new ArrayList<>(Arrays.asList(GREEN, PURPLE, YELLOW));
            colors.remove(c1);
            colors.remove(c2);
            return colors.get(0);
        }
    }

    public final int x, y;
    public Pixel mosaic = null;
    public final Color color;
    public double scoreValue = 0;

    public Pixel(int x, int y, Color color) {
        this.x = x;
        this.y = y;
        this.color = color;
    }

    public Pixel(Pixel p, Color color) {
        this(p.x, p.y, color);
        this.scoreValue = p.scoreValue;
    }

    public Pixel(Pixel p) {
        this(p, p.color);
    }

    public boolean isColored() {
        return color == Color.PURPLE || color == Color.YELLOW || color == Color.GREEN;
    }

    public boolean isEmpty() {
        return color == Color.EMPTY;
    }

    public boolean inMosaic() {
        return mosaic != null && mosaic.color != Color.INVALID;
    }

    public String toString() {
        double decPlaces = 100000;
        return "(" + x + ", " + y + "), " + color.name() + ", " + (int) (scoreValue * decPlaces) / decPlaces;
    }

    public void print() {
        System.out.println(this);
    }
}
