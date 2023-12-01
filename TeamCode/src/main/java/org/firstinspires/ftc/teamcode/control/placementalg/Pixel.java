package org.firstinspires.ftc.teamcode.control.placementalg;

import static org.firstinspires.ftc.teamcode.control.placementalg.Pixel.Color.*;
import static org.firstinspires.ftc.teamcode.control.placementalg.ScoringMeasurements.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;
import java.util.Arrays;

public final class Pixel implements Comparable<Pixel> {

    public int compareTo(Pixel cPixel) {
        double diff = cPixel.scoreValue - scoreValue;
        if (diff == 0) diff = y - cPixel.y;
        if (diff == 0) diff = x - cPixel.x;
        return (int) (diff * 1000000000);
    }

    @Config
    public enum Color {
        INVALID,
        EMPTY,
        ANY,
        COLORED,
        WHITE,
        PURPLE,
        GREEN,
        YELLOW;

        public static float TEST_VAR = 0;

        private static final String RESET = "\u001B[0m";

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
                case EMPTY:
                default:
                    return "_";
            }
        }

        public static Color fromString(String color) {
            switch (color.toUpperCase()) {
                case "W":
                    return WHITE;
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

        public static Color fromHSV(float[] hsv) {
            return EMPTY;
        }

        public static Color getRemainingColor(Color c1, Color c2) {
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

        public boolean isColored() {
            return this == PURPLE || this == YELLOW || this == GREEN;
        }

        public boolean isEmpty() {
            return this == EMPTY;
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

    public boolean inMosaic() {
        return mosaic != null && mosaic.color != INVALID;
    }

    public Pose2d toPose2d(boolean isRed) {
        return new Pose2d(
                SCORING_X,
                (isRed ? SCORING_Y_RED_HIGHEST : SCORING_Y_BLUE_HIGHEST) - (x * PIXEL_WIDTH) + (y % 2 == 0 ? 0.5 * PIXEL_WIDTH : 0),
                Math.toRadians(0)
        );
    }

    @NonNull
    public String toString() {
        double decPlaces = 100000;
        return "(" + x + ", " + y + "), " + color.name() + ", " + (int) (scoreValue * decPlaces) / decPlaces;
    }

    public void print() {
        System.out.println(this);
    }
}
