package org.firstinspires.ftc.teamcode.control.placementalg;

import java.util.ArrayList;
import java.util.Collections;

public class Backdrop {
    private final static int rows = 11;
    private final static int columns = 7;
    public boolean noColor = false;

    private final Pixel[][] slots = new Pixel[rows][columns];
    public final ArrayList<Pixel> pixelsToPlace = new ArrayList<>();
    private final ArrayList<Pixel> colorsToGetSPixels = new ArrayList<>();
    private ArrayList<Pixel> setLineSPixels;
    private Pixel setLineGoal;

    public int numOfMosaics = 0;
    public static final Backdrop PERFECT_BACKDROP;

    static {
        PERFECT_BACKDROP = new Backdrop();
        PERFECT_BACKDROP.calculate();
        while (!PERFECT_BACKDROP.rowFull(10)) {
            Pixel pToPlace = PERFECT_BACKDROP.pixelsToPlace.get(0);
            if (pToPlace.color == Pixel.Color.ANY)
                pToPlace = new Pixel(pToPlace, Pixel.Color.COLORED);
            PERFECT_BACKDROP.add(pToPlace);
            PERFECT_BACKDROP.calculate();
        }
    }

    public Backdrop() {
        for (int y = 0; y < slots.length; y++) for (int x = 0; x < slots[y].length; x++) {
            slots[y][x] = new Pixel(x, y, (y % 2 == 0 && x == 0) ? Pixel.Color.INVALID : Pixel.Color.EMPTY);
        }
        setLineGoal = getSetLineGoal();
        setLineSPixels = getSupportPixels(setLineGoal);
    }

    public void add(Pixel pixel) {
        Pixel.Color[] colors = {
                Pixel.Color.PURPLE,
                Pixel.Color.GREEN,
                Pixel.Color.YELLOW,
                Pixel.Color.WHITE,
        };
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
                if (coordsInRange(x, y) && get(x, y).color != Pixel.Color.INVALID) slots[y][x] = pixel;
        }
    }

    public Pixel get(int x, int y) {
        return coordsInRange(x, y) ? slots[y][x] : new Pixel(x, y, Pixel.Color.INVALID);
    }

    private static boolean coordsInRange(int x, int y) {
        return x >= 0 && x < columns && y >= 0 && y < rows;
    }

    private boolean preferUpMosaic(Pixel pixel) {
        int x = pixel.x;
        int y = pixel.y;
        if (!(x == 3 && (y == 3 || y == 9))) return false;
        if (y == 9) return preferUpMosaic(new Pixel(x, 3, pixel.color));

        Pixel[] shouldBeColored = {
                get(5, y),
                get(6, y),
                get(6, y + 1),
        };
        for (Pixel p : shouldBeColored) if (!(p.isEmpty() || p.isColored())) return false;

        Pixel[] shouldBeWhite = {
                get(5, y - 1),
                get(6, y - 1),
        };
        for (Pixel p : shouldBeWhite) if (!(p.isEmpty() || p.color == Pixel.Color.WHITE)) return false;

        return true;
    }

    private Pixel[][] getPossibleMosaics(Pixel pixel) {
        int x = pixel.x;
        int y = pixel.y;
        Pixel[] up = {
                pixel,
                get(x, y + 1),
                get(x - 1 + 2 * (y % 2), y + 1)
        };
        Pixel[] right = {
                pixel,
                get(x + 1, y),
                get(x + (y % 2), y + 1)
        };
        Pixel[] left = {
                pixel,
                get(x - 1, y),
                get(x + (y % 2) - 1, y + 1)
        };
        if (x == 1) return new Pixel[][]{up, left, right};
        if (x == 6 || preferUpMosaic(pixel)) return new Pixel[][]{up, right, left};
        if (x == 5 || x == 3) return new Pixel[][]{right, left, up};
        return new Pixel[][]{left, right, up};
    }

    private void scanForMosaics() {
        for (Pixel[] row : slots) for (Pixel pixel : row) pixel.mosaic = null;
        for (int y = 0; y < slots.length && !rowEmpty(y); y++) {
            for (int x = 0; x < slots[x].length; x++) {

                Pixel pixel = get(x, y);
                if (pixel.mosaic != null || !pixel.isColored() || (pixel.isColored() && touchingAdjacentMosaic(pixel, false)))
                    continue;

                Pixel[][] possibleMosaics = getPossibleMosaics(pixel);
                Pixel[] pixels = {};

                boolean isMosaic = false;
                for (Pixel[] pMosaic : possibleMosaics) {
                    boolean sameColor = pMosaic[0].color == pMosaic[1].color && pMosaic[1].color == pMosaic[2].color;
                    boolean diffColors = pMosaic[0].color != pMosaic[1].color && pMosaic[1].color != pMosaic[2].color && pMosaic[0].color != pMosaic[2].color;
                    boolean allColored = pMosaic[1].isColored() && pMosaic[2].isColored();
                    isMosaic = sameColor || (diffColors && allColored);
                    if (isMosaic) {
                        pixels = pMosaic;
                        break;
                    }
                }

                if (isMosaic) {
                    for (Pixel p : pixels) p.mosaic = pixel;
                    for (Pixel p : pixels) {
                        if (touchingAdjacentMosaic(p, false)) {
                            invalidateMosaic(p.mosaic);
                            break;
                        }
                    }
                    if (pixels[0].mosaic.color != Pixel.Color.INVALID) numOfMosaics++;
                    continue;
                }

                for (Pixel[] pMosaic : possibleMosaics) {
                    for (Pixel a : pMosaic) if (a.isColored() || a.isEmpty()) a.mosaic = pixel;

                    if (
                            !touchingAdjacentMosaic(pMosaic[0], false) &&
                                    !touchingAdjacentMosaic(pMosaic[1], false) &&
                                    !touchingAdjacentMosaic(pMosaic[2], false)
                    ) {
                        if (pMosaic[1].isColored() && pMosaic[2].isEmpty()) {
                            Pixel b = pMosaic[2];
                            pixelsToPlace.add(new Pixel(b, Pixel.Color.getRemainingColor(pixel.color, pMosaic[1].color)));
                            b = new Pixel(b);
                            b.scoreValue += 11;
                            colorsToGetSPixels.add(b);
                        } else if (pMosaic[2].isColored() && pMosaic[1].isEmpty()) {
                            Pixel b = pMosaic[1];
                            pixelsToPlace.add(new Pixel(b, Pixel.Color.getRemainingColor(pixel.color, pMosaic[2].color)));
                            b = new Pixel(b);
                            b.scoreValue += 11;
                            colorsToGetSPixels.add(b);
                        } else if (pMosaic[1].isEmpty() && pMosaic[2].isEmpty()) {
                            pixelsToPlace.add(new Pixel(pMosaic[1], Pixel.Color.COLORED));
                            pixelsToPlace.add(new Pixel(pMosaic[2], Pixel.Color.COLORED));
                            Pixel p1 = new Pixel(pMosaic[1]);
                            Pixel p2 = new Pixel(pMosaic[2]);
                            p1.scoreValue += 22/3.0;
                            p2.scoreValue += 22/3.0;
                            colorsToGetSPixels.add(p1);
                            colorsToGetSPixels.add(p2);
                        }
                        if (pMosaic[1].isEmpty() || pMosaic[2].isEmpty()) {
                            invalidateMosaic(pixel);
                            break;
                        }
                    }
                    invalidateMosaic(pixel);

                }
            }
        }
        for (Pixel p : colorsToGetSPixels) pixelsToPlace.addAll(getSupportPixels(p));
        removeDuplicates(pixelsToPlace);
        removeUnsupportedPixels(pixelsToPlace);
        removeOverridingPixels(pixelsToPlace);
    }

    public String toString() {
        String spacer = " ";
        StringBuilder backdrop = new StringBuilder();
        for (int y = rows - 1; y >= 0; y--) {
            backdrop.append(y).append(spacer).append(y >= 10 ? "" : spacer);
            for (int x = 0; x < columns; x++) {
                Pixel pixel = get(x, y);
                String color = pixel.color.toString();
                if (pixel.inMosaic()) color = color.toLowerCase();
                backdrop.append(color).append(x == columns - 1 ? "" : spacer);
            }
            backdrop.append('\n');
        }
        backdrop.append(spacer).append(spacer).append(spacer);
        for (int x = 0; x < columns; x++) backdrop.append(x).append(x == columns - 1 ? "" : spacer);
        return backdrop.toString();
    }

    public void print() {
        System.out.println(this);
        System.out.println();
        printPixelsToPlace();
    }

    private void updateScoreValues() {
        for (Pixel pixel : pixelsToPlace) {
            if (!noColor) {
                if (pixel.isColored()) pixel.scoreValue += 11;
                if (pixel.color == Pixel.Color.COLORED) pixel.scoreValue += 22/3.0;
                if (pixel.color == Pixel.Color.WHITE) pixel.scoreValue += 11/9.0;
                for (Pixel mosaicPixel : colorsToGetSPixels) {
                    ArrayList<Pixel> mosaicSPixels = getSupportPixels(mosaicPixel);
                    if (inArray(pixel, mosaicSPixels)) {
                        pixel.scoreValue += mosaicPixel.scoreValue / (double) mosaicSPixels.size();
                    }
                }
            }

            if (inArray(pixel, setLineSPixels)) pixel.scoreValue += 10 / (double) setLineSPixels.size();
        }
        Collections.sort(pixelsToPlace);
    }

    private void removeUnsupportedPixels(ArrayList<Pixel> pixels) {
        ArrayList<Pixel> pixelsCopy = new ArrayList<>(pixels);
        pixels.clear();
        for (Pixel pixel : pixelsCopy) if (isSupported(pixel)) pixels.add(pixel);
    }

    private void removeDuplicates(ArrayList<Pixel> pixels) {
        ArrayList<Pixel> pixelsCopy = new ArrayList<>(pixels);
        pixels.clear();
        for (Pixel pixel : pixelsCopy) if (!inArray(pixel, pixels)) pixels.add(pixel);
    }

    private void removeOverridingPixels(ArrayList<Pixel> pixels) {
        ArrayList<Pixel> pixelsCopy = new ArrayList<>(pixels);
        pixels.clear();
        for (Pixel pixel : pixelsCopy) if (get(pixel.x, pixel.y).isEmpty()) pixels.add(pixel);
    }

    private void invalidateMosaic(Pixel mosaic) {
        Pixel invMosaic = new Pixel(mosaic, Pixel.Color.INVALID);
        for (int y = 0; y < slots.length && !rowEmpty(y); y++) for (Pixel pixel : slots[y]) {
            if (pixel.mosaic == mosaic && (pixel.isColored() || pixel.isEmpty()))
                pixel.mosaic = invMosaic;
        }

        for (int y = 0; y < slots.length && !rowEmpty(y); y++) for (Pixel pixel : slots[y]) {
            if (pixel.isColored() && touchingAdjacentMosaic(pixel, false)) pixel.mosaic = invMosaic;
        }
    }

    private boolean touchingAdjacentMosaic(Pixel pixel, boolean includeEmpties) {
        return getAdjacentMosaics(pixel, includeEmpties).size() > 0;
    }

    private ArrayList<Pixel> getAdjacentMosaics(Pixel pixel, boolean includeEmpties) {
        ArrayList<Pixel> adjacentMosaics = new ArrayList<>();
        for (Pixel neighbor : getNeighbors(pixel, includeEmpties)) {
            if ((neighbor.isColored() || neighbor.isEmpty()) && neighbor.mosaic != pixel.mosaic) {
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
            if (includeEmpties || !n.isEmpty())
                neighbors.add(n);
        }
        return neighbors;
    }

    private boolean isSupported(Pixel pixel) {
        return !get(pixel.x, pixel.y - 1).isEmpty() && !get(pixel.x - 1 + 2 * (pixel.y % 2), pixel.y - 1).isEmpty();
    }

    public ArrayList<Pixel> getSupportPixels(Pixel pixel) {
        ArrayList<Pixel> sPixels = new ArrayList<>();
        if (pixel.isEmpty()) {
            sPixels.add(getSafeColor(pixel));
            Pixel s1 = get(pixel.x, pixel.y - 1);
            Pixel s2 = get(pixel.x - 1 + 2 * (pixel.y % 2), pixel.y - 1);
            sPixels.addAll(getSupportPixels(s1));
            sPixels.addAll(getSupportPixels(s2));
        }
        removeDuplicates(sPixels);
        removeOverridingPixels(sPixels);
        return sPixels;
    }

    private Pixel getSetLineGoal() {
        int highestY = getHighestPixelY();
        int setY = highestY >= 5 ? 8 :
                highestY >= 2 ? 5 :
                        2;

        int leastSPixels = 100;
        Pixel bestSetPixel = get(6, 8);

        for (int x = 0; x < columns; x++) {
            Pixel pixel = get(x, setY);
            if (pixel.color == Pixel.Color.INVALID) continue;
            ArrayList<Pixel> sPixels = getSupportPixels(pixel);
            if (sPixels.size() < leastSPixels) {
                leastSPixels = sPixels.size();
                bestSetPixel = pixel;
            }
        }

        return bestSetPixel;
    }

    private int getHighestPixelY() {
        int highestY = 0;
        for (int y = 0; y < slots.length && !rowEmpty(y); y++) for (Pixel p : slots[y]) {
            if (!p.isEmpty() && p.color != Pixel.Color.INVALID) highestY = p.y;
        }
        return highestY;
    }

    private boolean inArray(Pixel pixel, Iterable<Pixel> array) {
        for (Pixel p1 : array) if (pixel.x == p1.x && pixel.y == p1.y) return true;
        return false;
    }

    private boolean colorInArray(Pixel.Color color, Iterable<Pixel> array) {
        for (Pixel p1 : array) if (color == p1.color) return true;
        return false;
    }

    private void scanForEmptySpot() {
        for (int y = 0; y < slots.length; y++) {
            for (int x = 0; x < slots[x].length; x++) {
                Pixel pixel = get(x, y);
                if (pixel.isEmpty() && pixel.color != Pixel.Color.INVALID && !inArray(pixel, pixelsToPlace) && isSupported(pixel)) {
                    pixelsToPlace.add(getSafeColor(pixel));
                    return;
                }
            }
        }
    }

    private Pixel getSafeColor(Pixel pixel) {
        return new Pixel(pixel, touchingAdjacentMosaic(pixel, false) || noSpaceForMosaics(pixel) ? Pixel.Color.WHITE : Pixel.Color.ANY);
    }

    private static boolean allTrue(boolean... booleans) {
        for(boolean b : booleans) if(!b) return false;
        return true;
    }

    private boolean noSpaceForMosaics(Pixel pixel) {
        Pixel[][] pMosaics = getPossibleMosaics(pixel);
        boolean[] pMosaicsBlocked = new boolean[pMosaics.length];
        for (int i = 0; i < pMosaics.length; i++) {
            if (!pMosaics[i][1].isEmpty() || !pMosaics[i][2].isEmpty() || touchingAdjacentMosaic(pMosaics[i][1], true) || touchingAdjacentMosaic(pMosaics[i][2], true)) {
                pMosaicsBlocked[i] = true;
            }
        }
        return allTrue(pMosaicsBlocked);
    }

    private boolean rowEmpty(int y) {
        for (Pixel pixel : slots[y]) if (!pixel.isEmpty() && pixel.color != Pixel.Color.INVALID) return false;
        return true;
    }

    public boolean rowFull(int y) {
        for (Pixel pixel : slots[y]) if (pixel.isEmpty()) return false;
        return true;
    }

    public void printPixelsToPlace() {
        for (Pixel pixel : pixelsToPlace) pixel.print();
    }

    private void scanForSetLinePixels() {
        setLineGoal = getSetLineGoal();
        setLineSPixels = getSupportPixels(setLineGoal);
        if (setLineGoal.y <= 8) pixelsToPlace.addAll(setLineSPixels);
        removeUnsupportedPixels(pixelsToPlace);
    }

    public void calculate() {
        pixelsToPlace.clear();
        colorsToGetSPixels.clear();
        numOfMosaics = 0;

        scanForMosaics();
        scanForSetLinePixels();
        scanForEmptySpot();
        for (int i = 0; i < 7 && !colorInArray(Pixel.Color.ANY, pixelsToPlace); i++) scanForEmptySpot();

        removeDuplicates(pixelsToPlace);
        removeOverridingPixels(pixelsToPlace);
        removeUnsupportedPixels(pixelsToPlace);

        updateScoreValues();
    }

}
