package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.INVALID;
import static java.lang.Math.random;
import static java.lang.Math.round;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import java.util.Arrays;

@Config
public final class Backdrop {

    public static double
            BOTTOM_ROW_HEIGHT = 4,
            X = 47,
            Y_MAX_BLUE = 45,
            Y_MAX_RED = -25;

    final static int ROWS = 11, COLUMNS = 7;
    public boolean printRectangular = true;

    final Pixel[][] slots = new Pixel[ROWS][COLUMNS];

    int mosaicCount = 0;

    Backdrop() {
        for (int y = 0; y < slots.length; y++)
            for (int x = 0; x < slots[y].length; x++) {
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

    public void add(Pixel pixel) {
        switch (pixel.color) {
            case ANY:
                add(new Pixel(pixel, Pixel.Color.get((int) round(random() * 3) + 1)));
                break;
            case COLORED:
                add(new Pixel(pixel, Pixel.Color.get((int) round(random() * 2) + 2)));
                break;
            default:
                int x = pixel.x;
                int y = pixel.y;
                if (coordsInRange(x, y) && !(y % 2 == 0 && x == 0)) slots[y][x] = pixel;
        }
    }

    Pixel get(int x, int y) {
        return coordsInRange(x, y) ? slots[y][x] : new Pixel(x, y, INVALID);
    }

    Pixel get(Pixel pixel) {
        return get(pixel.x, pixel.y);
    }

    private static boolean coordsInRange(int x, int y) {
        return x >= 0 && x < COLUMNS && y >= 0 && y < ROWS;
    }

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

    void print() {
        System.out.println(this);
        System.out.println();
        System.out.println("Auton score: " + getPixelCount() * 5);
        System.out.println("Teleop score: " + (getPixelCount() * 3 + (mosaicCount + getSetLinesReached()) * 10));
    }

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

    boolean touching(Pixel p1, Pixel p2) {
        return inArray(get(p1), Arrays.asList(getNeighbors(get(p2))));
    }

    boolean isSupported(Pixel pixel) {
        get(pixel.x - 1 + 2 * (pixel.y % 2), pixel.y - 1);
        get(pixel.x, pixel.y - 1);
        return !(get(pixel.x, pixel.y - 1).color == EMPTY) && !(get(pixel.x - 1 + 2 * (pixel.y % 2), pixel.y - 1).color == EMPTY);
    }

    int getHighestPixelY() {
        int highestY = 0;
        for (Pixel[] row : slots)
            for (Pixel p : row) {
                if (!(p.color == EMPTY) && p.color != INVALID) highestY = p.y;
            }
        return highestY;
    }

    static boolean inArray(Pixel pixel, Iterable<Pixel> array) {
        for (Pixel p1 : array) if (pixel.x == p1.x && pixel.y == p1.y) return true;
        return false;
    }

    static boolean allTrue(boolean... booleans) {
        for (boolean b : booleans) if (!b) return false;
        return true;
    }

    public boolean notFull() {
        for (Pixel pixel : slots[10]) if (pixel.color == EMPTY) return true;
        return false;
    }

    private int getPixelCount() {
        int pixelCount = 0;
        for (Pixel[] row : slots) {
            for (Pixel pixel : row) {
                if (!(pixel.color == EMPTY || pixel.color == INVALID)) pixelCount++;
            }
        }
        return pixelCount;
    }

    private int getSetLinesReached() {
        return (getHighestPixelY() + 1) / 3;
    }
}
