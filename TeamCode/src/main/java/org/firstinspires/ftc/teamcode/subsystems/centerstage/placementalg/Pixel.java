package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRed;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.INVALID;
import static java.lang.Math.PI;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;

@Config
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
    final Pixel.Color color;
    double scoreValue = 0;

    Pixel(int x, int y, Pixel.Color color) {
        this.x = x;
        this.y = y;
        this.color = color;
    }

    Pixel(Pixel p, Pixel.Color color) {
        this(p.x, p.y, color);
        this.scoreValue = p.scoreValue;
    }

    @NonNull
    protected Pixel clone() {
        return new Pixel(this, color);
    }

    boolean inMosaic() {
        return mosaic != null && mosaic.color != INVALID;
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

    @NonNull
    public String toString() {
        double decPlaces = 100000;
        return "(" + x + ", " + y + "), " + color.name() + ", " + (int) (scoreValue * decPlaces) / decPlaces;
    }

    void print() {
        System.out.println(this);
    }

    public static HSV
            minWhite = new HSV(
            0,
            0,
            0.05
            ),
            maxWhite = new HSV(
                    0,
                    0.6,
                    0.45
            ),
            minPurple = new HSV(
                    205,
                    0.55,
                    0.085
            ),
            maxPurple = new HSV(
                    225,
                    1,
                    0.35
            ),
            minYellow = new HSV(
                    90,
                    0.55,
                    0.02
            ),
            maxYellow = new HSV(
                    125,
                    1,
                    0.15
            ),
            minGreen = new HSV(
                    130,
                    0.5,
                    0.01
            ),
            maxGreen = new HSV(
                    160,
                    1,
                    0.2
            );

    static boolean printInColor = true;

    @Config
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

        @NonNull
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

        public static Color fromHSV(HSV hsv) {
            return hsv.inRange(minPurple, maxPurple) ? PURPLE :
                    hsv.inRange(minGreen, maxGreen) ? GREEN :
                    hsv.inRange(minYellow, maxYellow) ? YELLOW :
                    new HSV(0, hsv.saturation, hsv.value).inRange(minWhite, maxWhite) ? WHITE :
                    EMPTY;
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
