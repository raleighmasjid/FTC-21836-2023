package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.MeepMeepTesting.isRed;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public final class Pixel implements Comparable<Pixel> {

    public static double
            WIDTH = 3,
            HEIGHT = 2.59945;

    public int compareTo(Pixel other) {
        double diff = other.scoreValue - scoreValue;
        if (diff == 0) diff = y - other.y;
        return (int) (diff * 1000000000);
    }

    final int x, y;
    Pixel mosaic = null;
    final Color color;
    double scoreValue = 0;

    Pixel(int x, int y, Color color) {
        this.x = x;
        this.y = y;
        this.color = color;
    }

    Pixel(Pixel p, Color color) {
        this(p.x, p.y, color);
        this.scoreValue = p.scoreValue;
    }

    protected Pixel clone() {
        return new Pixel(this, color);
    }

    boolean inMosaic() {
        return mosaic != null && mosaic.color != Color.INVALID;
    }

    Pose2d toPose2d() {
        return new Pose2d(
                Backdrop.X,
                (isRed ? Backdrop.Y_MAX_RED : Backdrop.Y_MAX_BLUE) - (x * Pixel.WIDTH) + (y % 2 == 0 ? 0.5 * Pixel.WIDTH : 0),
                PI
        );
    }

    boolean isIn(Iterable<Pixel> array) {
        return getCounterpartIn(array) != null;
    }

    Pixel getCounterpartIn(Iterable<Pixel> array) {
        for (Pixel p1 : array) if (x == p1.x && y == p1.y) return p1;
        return null;
    }

    public String toString() {
        double decPlaces = 100000;
        return "(" + x + ", " + y + "), " + color.name() + ", " + (int) (scoreValue * decPlaces) / decPlaces;
    }

    void print() {
        System.out.println(this);
    }


    static boolean printInColor = true;

    public enum Color {
        PURPLE,
        YELLOW,
        GREEN,
        WHITE,
        EMPTY,
        ANY,
        ANYCOLOR,
        INVALID,
        HIGHLIGHTED;

        private static final String RESET = "\u001B[0m";
        private static final Color[] values = values();
        static Color get(int ordinal) {
            return values[ordinal];
        }

        public String toString() {
            switch (this) {
                case PURPLE:
                    if (printInColor) return "\u001B[35m" + "P" + RESET;
                case YELLOW:
                    if (printInColor) return "\u001B[33m" + "Y" + RESET;
                case GREEN:
                    if (printInColor) return "\u001B[32m" + "G" + RESET;
                case WHITE:
                case ANY:
                case ANYCOLOR:
                    return "" + name().charAt(0);
                case INVALID:
                    return " ";
                case HIGHLIGHTED:
                    return "#";
                case EMPTY:
                default:
                    return "_";
            }
        }

        static Color fromString(String color) {
            switch (color.toUpperCase()) {
                case "W":
                    return WHITE;
                case "#":
                case "A":
                    return ANY;
                case "C":
                    return ANYCOLOR;
                case "P":
                    return PURPLE;
                case "Y":
                    return YELLOW;
                case "G":
                    return GREEN;
                case " ":
                    return INVALID;
                case "_":
                default:
                    return EMPTY;
            }
        }

        boolean matches(Color other) {
            return (this != INVALID && other != INVALID) && (
                    this == ANY ||
                            other == ANY ||
                            this == other ||
                            isColored() && other == ANYCOLOR ||
                            this == ANYCOLOR && other.isColored()
            );
        }

        boolean isColored() {
            return ordinal() <= 2;
        }
    }
}
