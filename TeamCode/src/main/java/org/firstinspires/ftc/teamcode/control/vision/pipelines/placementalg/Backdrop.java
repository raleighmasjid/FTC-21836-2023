package org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg;

import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.INVALID;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.printInColor;
import static java.lang.Math.random;
import static java.lang.Math.round;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public final class Backdrop {

    public static double
            BOTTOM_ROW_HEIGHT = 4,
            X = 49.5,
            Y_MAX_BLUE = 44.25,
            Y_MAX_RED = -26.25;

    final static int ROWS = 11, COLUMNS = 7;
    public boolean printRectangular = false;

    final Pixel[][] slots = new Pixel[ROWS][COLUMNS];

    public int mosaicCount = 0;

    public Backdrop() {
        for (int y = 0; y < slots.length; y++) for (int x = 0; x < slots[y].length; x++) {
            slots[y][x] = new Pixel(x, y, (y % 2 == 0 && x == 0) ? INVALID : EMPTY);
        }
    }

    @NonNull
    public Backdrop clone() {
        Backdrop backdrop = new Backdrop();
        backdrop.mosaicCount = mosaicCount;
        backdrop.printRectangular = printRectangular;
        for (Pixel[] row : slots) for (Pixel pixel : row) backdrop.add(pixel);
        return backdrop;
    }

    public boolean equals(Object other) {
        if (!(other instanceof Backdrop)) return false;
        Backdrop other1 = (Backdrop) other;
        for (Pixel[] row : slots) for (Pixel pixel : row) {
            if (other1.get(pixel).color != pixel.color) return false;
        }
        return true;
    }

    /**
     * Clears every slot in the backdrop, as if it were a new backdrop <br>
     * Also resets {@link #mosaicCount} to 0
     */
    public void clear() {
        for (int y = 0; y < slots.length; y++) for (int x = 0; x < slots[y].length; x++) {
            slots[y][x] = new Pixel(x, y, (y % 2 == 0 && x == 0) ? INVALID : EMPTY);
        }
        mosaicCount = 0;
    }

    /**
     * Add a {@link Pixel} to the {@link Backdrop} based on its <br>
     * If provided a {@link Pixel} of {@link Pixel.Color} ANY or ANYCOLOR, this method will randomly choose a specified version of the provided {@link Pixel.Color}
     * @param pixel The {@link Pixel} to add
     * @return This {@link Backdrop}
     */
    public Backdrop add(Pixel pixel) {
        switch (pixel.color) {
            case ANY:
                add(new Pixel(pixel, Pixel.Color.get((int) round(random() * 3))));
                break;
            case ANYCOLOR:
                add(new Pixel(pixel, Pixel.Color.get((int) round(random() * 2))));
                break;
            default:
                int x = pixel.x;
                int y = pixel.y;
                if (coordsInRange(x, y) && !(y % 2 == 0 && x == 0)) slots[y][x] = pixel;
        }
        return this;
    }

    /**
     * @return The {@link Pixel} found at the provided x and y coordinates, if {@link #coordsInRange}
     */
    public Pixel get(int x, int y) {
        return coordsInRange(x, y) ? slots[y][x] : new Pixel(x, y, INVALID);
    }

    /**
     * @return The {@link Pixel} found at the corresponding location of the provided {@link Pixel}, if {@link #coordsInRange}
     */
    Pixel get(Pixel pixel) {
        return get(pixel.x, pixel.y);
    }

    /**
     * @return Whether the provided x, y location is within the bounds of this {@link Backdrop} object
     */
    private static boolean coordsInRange(int x, int y) {
        return x >= 0 && x < COLUMNS && y >= 0 && y < ROWS;
    }

    /**
     * @return A multi-line {@link String} representation of the {@link Pixel} contents of this {@link Backdrop} object
     */
    @NonNull
    public String toString() {
        String spacer = " ";
        StringBuilder backdrop = new StringBuilder();
        for (int y = ROWS - 1; y >= 0; y--) {
            backdrop.append(y).append(spacer).append(y >= 10 ? "" : spacer);
            for (int x = 0; x < COLUMNS; x++) {
                Pixel pixel = get(x, y);
                String color = pixel.color.toString();
                if (pixel.inMosaic()) color = color.toLowerCase();
                if (!(y % 2 == 0 && x == 0) || printRectangular) backdrop.append(color);
                backdrop.append(x == COLUMNS - 1 ? "" : spacer);
            }
            backdrop.append('\n');
        }
        backdrop.append(spacer).append(spacer).append(spacer);
        for (int x = 0; x < COLUMNS; x++) backdrop.append(x).append(x == COLUMNS - 1 ? "" : spacer);
        return backdrop.toString();
    }

    /**
     * Output the result of {@link #toString()}, {@link #mosaicCount}, and points scored to the main text output stream
     */
    public void print() {
        System.out.println(this);
        System.out.println();
        System.out.println(mosaicCount + " mosaics");
        System.out.println("Auton score: " + getPixelCount() * 5);
        System.out.println("Teleop score: " + (getPixelCount() * 3 + (mosaicCount + getSetLinesReached()) * 10));
    }

    /**
     * Output the result of {@link #toString()} to telemetry
     */
    public void toTelemetry(Telemetry mTelemetry) {
        printInColor = false;
        String[] rows = toString().split("\n");
        for (String row : rows) mTelemetry.addLine(row);
    }

    /**
     * @param pixel The {@link Pixel} to find the neighbors of
     * @return The 6 directly adjacent neighbor {@link Pixel} objects of the provided {@link Pixel}
     */
    Pixel[] getNeighbors(Pixel pixel) {
        int x = pixel.x;
        int y = pixel.y;
        return new Pixel[]{
                get(x + 1, y),
                get(x - 1, y),
                get(x, y + 1),
                get(x, y - 1),
                get(x - 1 + 2 * (y % 2), y + 1),
                get(x - 1 + 2 * (y % 2), y - 1),
        };
    }

    /**
     * @return Whether the two provided {@link Pixel}s are directly adjacent as per {@link #getNeighbors}
     */
    boolean touching(Pixel p1, Pixel p2) {
        return get(p1).isIn(Arrays.asList(getNeighbors(get(p2))));
    }

    /**
     * @return Whether the provided {@link Pixel} is supported by two other {@link Pixel}s below it in hexagonal grid space
     */
    boolean isSupported(Pixel pixel) {
        return get(pixel.x, pixel.y - 1).color != EMPTY && get(pixel.x - 1 + 2 * (pixel.y % 2), pixel.y - 1).color != EMPTY;
    }

    /**
     * @return The highest y-value of any {@link Pixel} on this {@link Backdrop}
     */
    int getHighestPixelY() {
        int highestY = 0;
        for (Pixel[] row : slots) for (Pixel p : row) {
            if (!(p.color == EMPTY) && p.color != INVALID) highestY = p.y;
        }
        return highestY;
    }

    /**
     * @return Whether ALL the provided booleans are true
     */
    static boolean allTrue(boolean... booleans) {
        for (boolean b : booleans) if (!b) return false;
        return true;
    }

    /**
     * @return Whether this {@link Backdrop} has any empty slots
     */
    public boolean notFull() {
        for (Pixel pixel : slots[10]) if (pixel.color == EMPTY) return true;
        return false;
    }

    /**
     * @return The number of pixels present in this {@link Backdrop}
     */
    private int getPixelCount() {
        int pixelCount = 0;
        for (Pixel[] row : slots) {
            for (Pixel pixel : row) {
                if (!(pixel.color == EMPTY || pixel.color == INVALID)) pixelCount++;
            }
        }
        return pixelCount;
    }

    /**
     * @return The number of "set lines" reached by the highest pixel on this {@link Backdrop}
     */
    private int getSetLinesReached() {
        return (getHighestPixelY() + 1) / 3;
    }
}
