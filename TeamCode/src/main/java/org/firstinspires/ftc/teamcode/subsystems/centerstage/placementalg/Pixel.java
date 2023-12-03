package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.INVALID;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.ScoringMeasurements.PIXEL_WIDTH;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.ScoringMeasurements.SCORING_X;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.ScoringMeasurements.SCORING_Y_BLUE_HIGHEST;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.ScoringMeasurements.SCORING_Y_RED_HIGHEST;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;

import java.util.ArrayList;
import java.util.Arrays;

public final class Pixel implements Comparable<Pixel> {

    public int compareTo(Pixel cPixel) {
        double diff = cPixel.scoreValue - scoreValue;
        if (diff == 0) diff = y - cPixel.y;
        if (diff == 0) diff = x - cPixel.x;
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
                SCORING_X,
                (isRed ? SCORING_Y_RED_HIGHEST : SCORING_Y_BLUE_HIGHEST) - (x * PIXEL_WIDTH) + (y % 2 == 0 ? 0.5 * PIXEL_WIDTH : 0),
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

        public static HSV
                minWhite = new HSV(
                        0,
                        0,
                        0
                ),
                maxWhite = new HSV(
                        1,
                        0.5F,
                        0.5F
                ),
                minPurple = new HSV(
                        0,
                        0,
                        0
                ),
                maxPurple = new HSV(
                        1,
                        0.5F,
                        0.5F
                ),
                minYellow = new HSV(
                        0,
                        0,
                        0
                ),
                maxYellow = new HSV(
                        1,
                        0.5F,
                        0.5F
                ),
                minGreen = new HSV(
                        0,
                        0,
                        0
                ),
                maxGreen = new HSV(
                        1,
                        0.5F,
                        0.5F
                );

        private static final String RESET = "\u001B[0m";
        private static final Color[] values = values();
        public static Color get(int ordinal) {
            return values[ordinal];
        }

        @NonNull
        public String toString() {
            switch (this) {
                case WHITE:
                    return "W";
                case ANY:
                    return "A";
                case COLORED:
                    return "C";
                case PURPLE:
                    return "\u001B[35m" + "P" + RESET;
                case YELLOW:
                    return "\u001B[33m" + "Y" + RESET;
                case GREEN:
                    return "\u001B[32m" + "G" + RESET;
                case INVALID:
                    return " ";
                case HIGHLIGHTED:
                    return "□";
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
                    hsv.inRange(minWhite, maxWhite) ? WHITE :
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

        public boolean isEmpty() {
            return this == EMPTY;
        }
    }
}
