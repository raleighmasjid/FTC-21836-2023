package org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg;

import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.INVALID;

import androidx.annotation.NonNull;

/**
 * Note: this class has a natural ordering that is not consistent with equals.
 */
public final class Pixel implements Comparable<Pixel> {

    /**
     * @return The difference in {@link #scoreValue} between this {@link Pixel} and the provided {@link Pixel}
     */
    public int compareTo(Pixel other) {
        double diff = other.scoreValue - scoreValue;
        if (diff == 0) diff = y - other.y;
        return (int) (diff * 1000000000);
    }

    public final int x;
    public final int y;
    public final Color color;
    double scoreValue = 0;
    public Pixel mosaic = null;

    public Pixel(int x, int y, Pixel.Color color) {
        this.x = x;
        this.y = y;
        this.color = color;
    }

    /**
     * Instantiate a new {@link Pixel} object based on an existing {@link Pixel} but with a new {@link Color}
     */
    public Pixel(Pixel p, Pixel.Color color) {
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

    public boolean equals(Object o) {
        if (!(o instanceof Pixel)) return false;
        Pixel p = (Pixel) o;
        return p.x == x && p.y == y;
    }

    /**
     * @return Whether this {@link Pixel} is part of a valid mosaic
     */
    public boolean inMosaic() {
        return mosaic != null && mosaic.color != INVALID;
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
     * Prints this scoring location to telemetry in an easily user-readable form
     */
    public String userFriendlyString() {
        return userFriendlyX() + ", " + color.name();
    }

    private String userFriendlyX() {
        switch (x) {
            case 0: return "FAR LEFT";
            case 1: return y % 2 == 0 ? "FAR LEFT" : "ALMOST FAR LEFT";
            case 2: return y % 2 == 0 ? "ALMOST FAR LEFT" : "LEFT OF CENTER";
            case 3: return y % 2 == 0 ? "CENTER LEFT" : "DEAD CENTER";
            case 4: return y % 2 == 0 ? "CENTER RIGHT" : "RIGHT OF CENTER";
            case 5: return "ALMOST FAR RIGHT";
            case 6: return "FAR RIGHT";
            default: return "UNKNOWN";
        }
    }

    /**
     * Outputs the result of {@link #toString()} to the main text output stream
     */
    public void print() {
        System.out.println(this);
    }

    static boolean printInColor = true;

    /**
     * An enum representing the color of a given {@link Pixel},
     * either a real, physical color, or a placeholder in a {@link Backdrop}
     */
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
        public static Color get(int ordinal) {
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
                case ANY: return "" + name().charAt(0);
                case ANYCOLOR: return "C";
                case INVALID: return " ";
                case EMPTY:
                default: return "_";
            }
        }

        public String humanInstruction() {
            switch (this) {
                case WHITE:
                case PURPLE:
                case YELLOW:
                case GREEN: return "" + (ordinal() + 1) % 4;
                default: return "";
            }
        }

        /**
         * @return The {@link Color} corresponding to a given {@link String}
         * representation of what is (likely) originally a {@link Color}
         */
        public static Color fromString(String color) {
            switch (color.toUpperCase()) {
                case "W": return WHITE;
                case "#":
                case "A": return ANY;
                case "C": return ANYCOLOR;
                case "P": return PURPLE;
                case "Y": return YELLOW;
                case "G": return GREEN;
                case " ": return INVALID;
                case "_":
                default: return EMPTY;
            }
        }

        /**
         * @return Whether this {@link Color} "matches" the provided {@link Color}, accounting for ambiguous and specific {@link Color}s
         */
        public boolean matches(Color other) {
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
