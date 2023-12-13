package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.INVALID;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public final class Pixel implements Comparable<Pixel> {

    public static double
            WIDTH = 3,
            HEIGHT = 2.59945;

    public int compareTo(Pixel other) {
        double diff = other.scoreValue - scoreValue;
        if (diff == 0) diff = y - other.y;
        if (diff == 0) diff = x - other.x;
        return (int) (diff * 1000000000);
    }

    final int x, y;
    Pixel mosaic = null;
    public final Pixel.Color color;
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

    Pixel(Pixel p) {
        this(p, p.color);
    }

    boolean inMosaic() {
        return mosaic != null && mosaic.color != INVALID;
    }

    public Pose2d toPose2d(boolean isRed) {
        return new Pose2d(
                Backdrop.X,
                (isRed ? Backdrop.Y_MAX_RED : Backdrop.Y_MAX_BLUE) - (x * Pixel.WIDTH) + (y % 2 == 0 ? 0.5 * Pixel.WIDTH : 0),
                toRadians(0)
        );
    }

    public int getY() {
        return y;
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
            0.15
            ),
            maxWhite = new HSV(
                    0,
                    0.6,
                    0.45
            ),
            minPurple = new HSV(
                    205,
                    0.55,
                    0.1
            ),
            maxPurple = new HSV(
                    225,
                    1,
                    0.35
            ),
            minYellow = new HSV(
                    90,
                    0.55,
                    0.08
            ),
            maxYellow = new HSV(
                    110,
                    1,
                    0.15
            ),
            minGreen = new HSV(
                    120,
                    0.7,
                    0.05
            ),
            maxGreen = new HSV(
                    140,
                    1,
                    0.2
            );

    @Config
    public enum Color {
        EMPTY,
        WHITE,
        PURPLE,
        YELLOW,
        GREEN,
        ANY,
        COLORED,
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
//                    return "\u001B[35m" + "P" + RESET;
                case YELLOW:
//                    return "\u001B[33m" + "Y" + RESET;
                case GREEN:
//                    return "\u001B[32m" + "G" + RESET;
                case WHITE:
                case ANY:
                case COLORED:
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
                    return COLORED;
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

        static Color getRemainingColor(Color c1, Color c2) {
            if (c1 == EMPTY || c2 == EMPTY) return COLORED;
            if (c1 == c2) return c1;
            ArrayList<Color> colors = new ArrayList<>(Arrays.asList(GREEN, PURPLE, YELLOW));
            colors.remove(c1);
            colors.remove(c2);
            return colors.get(0);
        }

        public boolean matches(Color other) {
            return (this != INVALID && other != INVALID) && (
                    this == ANY ||
                            other == ANY ||
                            this == other ||
                            isColored() && other == COLORED ||
                            this == COLORED && other.isColored()
            );
        }

        boolean isColored() {
            return this == PURPLE || this == YELLOW || this == GREEN;
        }
    }
}
