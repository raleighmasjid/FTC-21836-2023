package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.PixelColor.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.PixelColor.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.PixelColor.INVALID;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.PixelColor.PURPLE;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.PixelColor.WHITE;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.PixelColor.YELLOW;

import androidx.annotation.NonNull;

import java.util.ArrayList;

public final class Backdrop {
    final static int ROWS = 11;
    final static int COLUMNS = 7;
    public boolean printRectangular = false;

    final Pixel[][] slots = new Pixel[ROWS][COLUMNS];

    public int numOfMosaics = 0;

    public Backdrop() {
        for (int y = 0; y < slots.length; y++) for (int x = 0; x < slots[y].length; x++) {
            slots[y][x] = new Pixel(x, y, (y % 2 == 0 && x == 0) ? INVALID : EMPTY);
        }
    }

    public void add(Pixel pixel) {
        PixelColor[] colors = {PURPLE, GREEN, YELLOW, WHITE};
        switch (pixel.color) {
            case ANY:
                add(new Pixel(pixel, colors[(int) Math.floor(Math.random() * 4)]));
                break;
            case COLORED:
                add(new Pixel(pixel, colors[(int) Math.floor(Math.random() * 3)]));
                break;
            default:
                int x = pixel.x;
                int y = pixel.y;
                if (coordsInRange(x, y) && get(x, y).color != INVALID) slots[y][x] = pixel;
        }
    }

    public Pixel get(int x, int y) {
        return coordsInRange(x, y) ? slots[y][x] : new Pixel(x, y, INVALID);
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
                if (!color.equals(" ") || printRectangular) backdrop.append(color);
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
    }

    boolean touchingAdjacentMosaic(Pixel pixel, boolean includeEmpties) {
        return getAdjacentMosaics(pixel, includeEmpties).size() > 0;
    }

    private ArrayList<Pixel> getAdjacentMosaics(Pixel pixel, boolean includeEmpties) {
        ArrayList<Pixel> adjacentMosaics = new ArrayList<>();
        for (Pixel neighbor : getNeighbors(pixel, includeEmpties)) {
            if ((neighbor.color.isColored() || neighbor.color.isEmpty()) && neighbor.mosaic != pixel.mosaic) {
                adjacentMosaics.add(neighbor);
            }
        }
        return adjacentMosaics;
    }

    private ArrayList<Pixel> getNeighbors(Pixel pixel, boolean includeEmpties) {
        ArrayList<Pixel> neighbors = new ArrayList<>();
        int x = pixel.x;
        int y = pixel.y;
        Pixel[] ns = {
                get(x + 1, y),
                get(x - 1, y),
                get(x, y + 1),
                get(x, y - 1),
                get(x - 1 + 2 * (y % 2), y + 1),
                get(x - 1 + 2 * (y % 2), y - 1),
        };
        for (Pixel n : ns) {
            if (includeEmpties || !n.color.isEmpty())
                neighbors.add(n);
        }
        return neighbors;
    }

    boolean isSupported(Pixel pixel) {
        return !get(pixel.x, pixel.y - 1).color.isEmpty() && !get(pixel.x - 1 + 2 * (pixel.y % 2), pixel.y - 1).color.isEmpty();
    }

    int getHighestPixelY() {
        int highestY = 0;
        for (int y = 0; y < slots.length && rowNotEmpty(y); y++) for (Pixel p : slots[y]) {
            if (!p.color.isEmpty() && p.color != INVALID) highestY = p.y;
        }
        return highestY;
    }

    static boolean inArray(Pixel pixel, Iterable<Pixel> array) {
        for (Pixel p1 : array) if (pixel.x == p1.x && pixel.y == p1.y) return true;
        return false;
    }

    static boolean allTrue(boolean... booleans) {
        for(boolean b : booleans) if(!b) return false;
        return true;
    }

    boolean rowNotEmpty(int y) {
        for (Pixel pixel : slots[y]) if (!pixel.color.isEmpty() && pixel.color != INVALID) return true;
        return false;
    }

    public boolean notFull() {
        for (Pixel pixel : slots[10]) if (pixel.color.isEmpty()) return true;
        return false;
    }

}
