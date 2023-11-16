package org.firstinspires.ftc.teamcode.control.placementalg;

import static org.firstinspires.ftc.teamcode.control.placementalg.ScoringMeasurements.PIXEL_BOTTOM_HEIGHT;
import static org.firstinspires.ftc.teamcode.control.placementalg.ScoringMeasurements.PIXEL_HEIGHT;
import static org.firstinspires.ftc.teamcode.control.placementalg.ScoringMeasurements.PIXEL_WIDTH;
import static org.firstinspires.ftc.teamcode.control.placementalg.ScoringMeasurements.SCORING_X;
import static org.firstinspires.ftc.teamcode.control.placementalg.ScoringMeasurements.SCORING_Y_BLUE_HIGHEST;
import static org.firstinspires.ftc.teamcode.control.placementalg.ScoringMeasurements.SCORING_Y_RED_HIGHEST;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;

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

        @NonNull
        public String toString() {
            switch (this) {
                case WHITE:
                    return "W";
                case ANY:
                    return "A";
                case COLORED:
                    return "C" + RESET;
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

        public boolean matches(Color other) {
            if (this == Color.INVALID || other == Color.INVALID) return false;
            return this == Color.ANY ||
                    other == Color.ANY ||
                    this == other ||
                    isColored() && other == Color.COLORED ||
                    this == Color.COLORED && other.isColored();
        }

        public boolean isColored() {
            return this == Color.PURPLE || this == Color.YELLOW || this == Color.GREEN;
        }

        public boolean isEmpty() {
            return this == Color.EMPTY;
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
        return mosaic != null && mosaic.color != Color.INVALID;
    }

    public Pose2d toPose2d(boolean isRed) {
        return new Pose2d(
                SCORING_X,
                (isRed ? SCORING_Y_RED_HIGHEST : SCORING_Y_BLUE_HIGHEST) - (x * PIXEL_WIDTH) + (y % 2 == 0 ? 0.5 * PIXEL_WIDTH : 0),
                Math.toRadians(0)
        );
    }

    public double toScoringHeight() {
        return PIXEL_BOTTOM_HEIGHT + (y * PIXEL_HEIGHT);
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
