package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop.allTrue;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop.inArray;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.ANY;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.COLORED;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.INVALID;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.WHITE;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.getRemainingColor;

import java.util.ArrayList;
import java.util.Collections;

public final class PlacementCalculator {
    private static Backdrop backdrop;
    private static final ArrayList<Pixel> pixelsToPlace = new ArrayList<>();
    private static final ArrayList<Pixel> colorsToGetSPixels = new ArrayList<>();
    private static ArrayList<Pixel> setLineSPixels;
    public static boolean noColor = false;
    public static final Backdrop PERFECT_BACKDROP;

    private PlacementCalculator() {}

    static {
        PERFECT_BACKDROP = new Backdrop();
        calculate(PERFECT_BACKDROP);
        while (PERFECT_BACKDROP.notFull()) {
            Pixel pToPlace = pixelsToPlace.get(0);
            if (pToPlace.color == ANY)
                pToPlace = new Pixel(pToPlace, COLORED);
            PERFECT_BACKDROP.add(pToPlace);
            calculate(PERFECT_BACKDROP);
        }
        PERFECT_BACKDROP.printRectangular = false;
    }

    private static boolean isSpecialCenterCase(Pixel pixel) {
        int x = pixel.x;
        int y = pixel.y;
        if (!(x == 3 && (y == 3 || y == 9))) return false;
        if (y == 9) return isSpecialCenterCase(new Pixel(x, 3, pixel.color));

        Pixel[] shouldBeColored = {
                backdrop.get(5, y),
                backdrop.get(6, y),
                backdrop.get(6, y + 1),
        };
        for (Pixel p : shouldBeColored) if (!(p.color == EMPTY || p.color.isColored())) return false;

        Pixel[] shouldBeWhite = {
                backdrop.get(5, y - 1),
                backdrop.get(6, y - 1),
        };
        for (Pixel p : shouldBeWhite) if (!(p.color == EMPTY || p.color == WHITE)) return false;

        return true;
    }

    private static Pixel[][] getPossibleMosaics(Pixel pixel) {
        int x = pixel.x;
        int y = pixel.y;
        Pixel[] up = {
                pixel,
                backdrop.get(x, y + 1),
                backdrop.get(x - 1 + 2 * (y % 2), y + 1)
        };
        Pixel[] right = {
                pixel,
                backdrop.get(x + 1, y),
                backdrop.get(x + (y % 2), y + 1)
        };
        Pixel[] left = {
                pixel,
                backdrop.get(x - 1, y),
                backdrop.get(x + (y % 2) - 1, y + 1)
        };
        if (x == 1) return new Pixel[][]{left, up, right};
        if (x == 6) return new Pixel[][]{right, up, left};
        if (isSpecialCenterCase(pixel)) return new Pixel[][]{up, right, left};
        if (x == 5 || x == 3) return new Pixel[][]{right, left, up};
        return new Pixel[][]{left, right, up};
    }

    private static void scanForMosaics() {
        for (Pixel[] row : backdrop.slots) for (Pixel pixel : row) pixel.mosaic = null;
        for (Pixel[] row : backdrop.slots) for (Pixel pixel : row) {

            if (pixel.mosaic != null || !pixel.color.isColored() || touchingAdjacentMosaic(pixel, false))
                continue;

            Pixel[][] possibleMosaics = getPossibleMosaics(pixel);
            Pixel[] pixels = {};

            boolean isMosaic = false;
            for (Pixel[] pMosaic : possibleMosaics) {
                boolean sameColor = pMosaic[0].color == pMosaic[1].color && pMosaic[1].color == pMosaic[2].color;
                boolean diffColors = pMosaic[0].color != pMosaic[1].color && pMosaic[1].color != pMosaic[2].color && pMosaic[0].color != pMosaic[2].color;
                boolean allColored = pMosaic[1].color.isColored() && pMosaic[2].color.isColored();
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
                if (pixels[0].mosaic.color != INVALID) backdrop.mosaicCount++;
                continue;
            }

            pMosaics:
            for (Pixel[] pMosaic : possibleMosaics) {
                for (Pixel a : pMosaic) if (a.color == WHITE) continue pMosaics;
                for (Pixel a : pMosaic) if (a.color.isColored() || a.color == EMPTY) a.mosaic = pixel;

                if (
                        !touchingAdjacentMosaic(pMosaic[0], false) &&
                                !touchingAdjacentMosaic(pMosaic[1], false) &&
                                !touchingAdjacentMosaic(pMosaic[2], false)
                ) {
                    if (pMosaic[1].color.isColored() && pMosaic[2].color == EMPTY) {
                        oneRemainingCase(pixel, pMosaic[2], pMosaic[1]);
                    } else if (pMosaic[2].color.isColored() && pMosaic[1].color == EMPTY) {
                        oneRemainingCase(pixel, pMosaic[1], pMosaic[2]);
                    } else if (pMosaic[1].color == EMPTY && pMosaic[2].color == EMPTY) {
                        pixelsToPlace.add(new Pixel(pMosaic[1], COLORED));
                        pixelsToPlace.add(new Pixel(pMosaic[2], COLORED));
                        Pixel p1 = new Pixel(pMosaic[1]);
                        Pixel p2 = new Pixel(pMosaic[2]);
                        p1.scoreValue += 22 / 3.0;
                        p2.scoreValue += 22 / 3.0;
                        colorsToGetSPixels.add(p1);
                        colorsToGetSPixels.add(p2);
                    }
                    if (pMosaic[1].color == EMPTY || pMosaic[2].color == EMPTY) {
                        invalidateMosaic(pixel);
                        if (pMosaic[1].color == EMPTY && pMosaic[2].color == EMPTY) {
                            Pixel invalid = new Pixel(pixel, INVALID);
                            for (Pixel[] bMosaic : possibleMosaics) for (Pixel a : bMosaic) a.mosaic = invalid;
                        }
                        break;
                    }
                }
                invalidateMosaic(pixel);

            }
        }
        for (Pixel p : colorsToGetSPixels) pixelsToPlace.addAll(getSupportPixels(p));
        removeDuplicates(pixelsToPlace);
        removeUnsupportedPixelsToPlace();
        removeOverridingPixels(pixelsToPlace);
    }

    private static void oneRemainingCase(Pixel pixel, Pixel x1, Pixel x2) {
        pixelsToPlace.add(new Pixel(x1, getRemainingColor(pixel.color, x2.color)));
        Pixel b = new Pixel(x1);
        b.scoreValue += 11;
        colorsToGetSPixels.add(b);
    }

    private static boolean touchingAdjacentMosaic(Pixel pixel, boolean includeEmpties) {
        for (Pixel neighbor : backdrop.getNeighbors(pixel)) {
            if (neighbor.mosaic != pixel.mosaic) {

                if (neighbor.color.isColored()) return true;

                if (includeEmpties && neighbor.color == EMPTY && neighbor.mosaic != null) {

                    Pixel[][] pMosaics = getPossibleMosaics(backdrop.get(neighbor.mosaic));
                    ArrayList<Pixel[]> activePMosaics = new ArrayList<>();
                    pMosaics:
                    for (Pixel[] pMosaic : pMosaics) {
                        for (Pixel p : pMosaic) if (p.mosaic == null) continue pMosaics;
                        activePMosaics.add(pMosaic);
                    }

                    boolean[] pMosaicsTouchingPixel = new boolean[activePMosaics.size()];

                    for (int i = 0; i < activePMosaics.size(); i++) {
                        for (Pixel p : activePMosaics.get(i)) {
                            if (backdrop.touching(p, pixel)) {
                                pMosaicsTouchingPixel[i] = true;
                                break;
                            }
                        }
                    }

                    if (allTrue(pMosaicsTouchingPixel)) return true;

                }
            }
        }
        return false;
    }

    private static void removeUnsupportedPixelsToPlace() {
        ArrayList<Pixel> pixelsToPlaceCopy = new ArrayList<>(pixelsToPlace);
        pixelsToPlace.clear();
        for (Pixel pixel : pixelsToPlaceCopy)
            if (backdrop.isSupported(pixel)) pixelsToPlace.add(pixel);
    }

    private static void removeDuplicates(ArrayList<Pixel> pixels) {
        ArrayList<Pixel> pixelsCopy = new ArrayList<>(pixels);
        pixels.clear();
        for (Pixel pixel : pixelsCopy) if (!inArray(pixel, pixels)) pixels.add(pixel);
    }

    private static void removeOverridingPixels(ArrayList<Pixel> pixels) {
        ArrayList<Pixel> pixelsCopy = new ArrayList<>(pixels);
        pixels.clear();
        for (Pixel pixel : pixelsCopy)
            if (backdrop.get(pixel).color == EMPTY) pixels.add(pixel);
    }

    private static void invalidateMosaic(Pixel mosaic) {
        Pixel invMosaic = new Pixel(mosaic, INVALID);
        for (Pixel[] row : backdrop.slots)
            for (Pixel pixel : row) {
                if (pixel.mosaic == mosaic && (pixel.color.isColored() || pixel.color == EMPTY)) {
                    pixel.mosaic = invMosaic;
                }
            }

        for (Pixel[] row : backdrop.slots)
            for (Pixel pixel : row) {
                if (pixel.color.isColored() && touchingAdjacentMosaic(pixel, false)) {
                    pixel.mosaic = invMosaic;
                }
            }
    }

    private static ArrayList<Pixel> getSupportPixels(Pixel pixel) {
        ArrayList<Pixel> sPixels = new ArrayList<>();
        if (pixel.color == EMPTY) {
            sPixels.add(getSafeColor(pixel));
            Pixel s1 = backdrop.get(pixel.x, pixel.y - 1);
            Pixel s2 = backdrop.get(pixel.x - 1 + 2 * (pixel.y % 2), pixel.y - 1);
            sPixels.addAll(getSupportPixels(s1));
            sPixels.addAll(getSupportPixels(s2));
        }
        removeDuplicates(sPixels);
        removeOverridingPixels(sPixels);
        return sPixels;
    }

    private static Pixel getSetLineGoal() {
        int highestY = backdrop.getHighestPixelY();
        int setY = highestY >= 5 ? 8 :
                highestY >= 2 ? 5 :
                        2;

        int leastSPixels = 100;
        Pixel bestSetPixel = backdrop.get(6, 8);

        for (int x = 0; x < Backdrop.COLUMNS; x++) {
            Pixel pixel = backdrop.get(x, setY);
            if (pixel.color == INVALID) continue;
            ArrayList<Pixel> sPixels = getSupportPixels(pixel);
            if (sPixels.size() < leastSPixels) {
                leastSPixels = sPixels.size();
                bestSetPixel = pixel;
            }
        }

        return bestSetPixel;
    }

    private static void scanForSetLinePixels() {
        Pixel setLineGoal = getSetLineGoal();
        setLineSPixels = getSupportPixels(setLineGoal);
        if (setLineGoal.y <= 8) pixelsToPlace.addAll(setLineSPixels);
        removeUnsupportedPixelsToPlace();
    }

    private static void scanForEmptySpot() {
        for (Pixel[] row : backdrop.slots) for (Pixel pixel : row) {
            if (pixel.color == EMPTY && pixel.color != INVALID && !inArray(pixel, pixelsToPlace) && backdrop.isSupported(pixel)) {
                pixelsToPlace.add(getSafeColor(pixel));
                return;
            }
        }
    }

    private static Pixel getSafeColor(Pixel pixel) {
        return new Pixel(pixel, touchingAdjacentMosaic(pixel, true) || noSpaceForMosaics(pixel) ? WHITE : ANY);
    }

    private static boolean noSpaceForMosaics(Pixel pixel) {
        Pixel[][] pMosaics = getPossibleMosaics(pixel);
        boolean[] pMosaicsBlocked = new boolean[pMosaics.length];
        for (int i = 0; i < pMosaics.length; i++) {
            if (!(pMosaics[i][1].color == EMPTY) || !(pMosaics[i][2].color == EMPTY) || touchingAdjacentMosaic(pMosaics[i][1], true) || touchingAdjacentMosaic(pMosaics[i][2], true)) {
                pMosaicsBlocked[i] = true;
            }
        }
        return allTrue(pMosaicsBlocked);
    }

    private static void sortPixelsToPlace() {
        for (Pixel pixel : pixelsToPlace) {
            if (!noColor) {
                if (pixel.color.isColored()) pixel.scoreValue += 11;
                if (pixel.color == COLORED) pixel.scoreValue += 22 / 3.0;
                if (pixel.color == WHITE) pixel.scoreValue += 11 / 9.0;
                for (Pixel mosaicPixel : colorsToGetSPixels) {
                    ArrayList<Pixel> mosaicSPixels = getSupportPixels(mosaicPixel);
                    if (inArray(pixel, mosaicSPixels)) {
                        pixel.scoreValue += mosaicPixel.scoreValue / (double) mosaicSPixels.size();
                    }
                }
            }

            if (inArray(pixel, setLineSPixels))
                pixel.scoreValue += 10 / (double) setLineSPixels.size();
        }
        Collections.sort(pixelsToPlace);
    }

    private static boolean willPlaceAny() {
        for (Pixel p1 : pixelsToPlace) if (p1.color == ANY) return true;
        return false;
    }

    public static ArrayList<Pixel> calculate(Backdrop backdrop) {
        PlacementCalculator.backdrop = backdrop;
        backdrop.mosaicCount = 0;
        pixelsToPlace.clear();
        colorsToGetSPixels.clear();

        scanForMosaics();
        scanForSetLinePixels();
        int i = 0;
        do scanForEmptySpot();
        while (++i < 8 && !willPlaceAny());

        removeDuplicates(pixelsToPlace);
        removeOverridingPixels(pixelsToPlace);
        removeUnsupportedPixelsToPlace();

        sortPixelsToPlace();
        return pixelsToPlace;
    }
}