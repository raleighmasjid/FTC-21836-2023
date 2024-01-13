package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRed;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.INVALID;
import static java.lang.Math.PI;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;

/**
 * Note: this class has a natural ordering that is not consistent with equals.
 */
@Config
public final class Pixel implements Comparable<Pixel> {

    public static double
            WIDTH = 3,
            HEIGHT = 2.59945;

    /**
     * @return The difference in {@link #scoreValue} between this {@link Pixel} and the provided {@link Pixel}
     */
    public int compareTo(Pixel other) {
        double diff = other.scoreValue - scoreValue;
        if (diff == 0) diff = y - other.y;
        return (int) (diff * 1000000000);
    }

    final int x;
    public final int y;
    final Color color;
    double scoreValue = 0;
    Pixel mosaic = null;

    Pixel(int x, int y, Pixel.Color color) {
        this.x = x;
        this.y = y;
        this.color = color;
    }

    /**
     * Instantiate a new {@link Pixel} object based on an existing {@link Pixel} but with a new {@link Color}
     */
    Pixel(Pixel p, Pixel.Color color) {
        this(p.x, p.y, color);
        this.scoreValue = p.scoreValue;
    }

    /**
     * @return A copy of this {@link Pixel}
     */
    @NonNull
    protected Pixel clone() {
        return new Pixel(this, color);
    }

    /**
     * @return Whether this {@link Pixel} is part of a valid mosaic
     */
    boolean inMosaic() {
        return mosaic != null && mosaic.color != INVALID;
    }

    /**
     * @return A {@link Pose2d} corresponding to the phsyical scoring location of this {@link Pixel}
     */
    public Pose2d toPose2d() {
        return new Pose2d(
                Backdrop.X,
                (isRed ? Backdrop.Y_MAX_RED : Backdrop.Y_MAX_BLUE) - (x * Pixel.WIDTH) + (y % 2 == 0 ? 0.5 * Pixel.WIDTH : 0),
                PI
        );
    }

    /**
     * @return Whether a {@link Pixel} with identical {@link #x} and {@link #y} is present in the provided {@link Iterable}
     */
    boolean isIn(Iterable<Pixel> array) {
        return getCounterpartIn(array) != null;
    }

    /**
     * @return The first {@link Pixel} with identical {@link #x} and {@link #y} present in the provided {@link Iterable} <br>
     * Returns null if no such {@link Pixel} is found
     */
    Pixel getCounterpartIn(Iterable<Pixel> array) {
        for (Pixel p1 : array) if (x == p1.x && y == p1.y) return p1;
        return null;
    }

    /**
     * @return A {@link String} representation of this {@link Pixel}, including its {@link #x}, {@link #y}, and {@link #scoreValue} to 5 decimal places
     */
    @NonNull
    public String toString() {
        double decPlaces = 100000;
        return "(" + x + ", " + y + "), " + color.name() + ", " + (int) (scoreValue * decPlaces) / decPlaces;
    }

    /**
     * Outputs the result of {@link #toString()} to the main text output stream
     */
    void print() {
        System.out.println(this);
    }

    /**
     * HSV value bound for intake pixel detection
     */
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

    /**
     * An enum representing the color of a given {@link Pixel},
     * either a real, physical color, or a placeholder in a {@link Backdrop}
     */
    @Config
    public enum Color {
        PURPLE,
        YELLOW,
        GREEN,
        WHITE,
        EMPTY,
        ANY,
        ANYCOLOR,
        INVALID;

        private static final String RESET = "\u001B[0m";
        private static final Color[] values = values();
        static Color get(int ordinal) {
            return values[ordinal];
        }

        /**
         * @return A single-letter {@link String} representation of this {@link Color} <br>
         * {@link #PURPLE}, {@link #GREEN}, and {@link #YELLOW} will have ANSI color codes if {@link #printInColor} is true
         */
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
                case EMPTY:
                default:
                    return "_";
            }
        }

        /**
         * @return The {@link Color} corresponding to a given {@link String}
         * representation of what is (likely) originally a {@link Color}
         */
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

        /**
         * @return The {@link Color} corresponding to the provided {@link HSV} as per the tuned value bounds
         */
        public static Color fromHSV(HSV hsv) {
            return hsv.between(minPurple, maxPurple) ? PURPLE :
                    hsv.between(minGreen, maxGreen) ? GREEN :
                    hsv.between(minYellow, maxYellow) ? YELLOW :
                    new HSV(0, hsv.saturation, hsv.value).between(minWhite, maxWhite) ? WHITE :
                    EMPTY;
        }

        /**
         * @return Whether this {@link Color} "matches" the provided {@link Color}, accounting for ambiguous and specific {@link Color}s
         */
        boolean matches(Color other) {
            return (this != INVALID && other != INVALID) && (
                    this == ANY ||
                            other == ANY ||
                            this == other ||
                            isColored() && other == ANYCOLOR ||
                            this == ANYCOLOR && other.isColored()
            );
        }

        /**
         * @return Whether this {@link Color} is {@link #PURPLE}, {@link #GREEN}, or {@link #YELLOW}
         */
        boolean isColored() {
            return ordinal() <= 2;
        }
    }
}
