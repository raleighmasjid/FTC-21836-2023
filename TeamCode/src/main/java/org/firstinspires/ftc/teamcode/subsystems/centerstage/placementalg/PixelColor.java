package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public enum PixelColor {
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

    static PixelColor fromString(String color) {
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

    public static PixelColor fromHSV(HSV hsv) {
        return EMPTY;
    }

    static PixelColor getRemainingColor(PixelColor c1, PixelColor c2) {
        if (c1 == EMPTY || c2 == EMPTY) return COLORED;
        if (c1 == c2) return c1;
        ArrayList<PixelColor> colors = new ArrayList<>(Arrays.asList(GREEN, PURPLE, YELLOW));
        colors.remove(c1);
        colors.remove(c2);
        return colors.get(0);
    }

    public boolean matches(PixelColor other) {
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
