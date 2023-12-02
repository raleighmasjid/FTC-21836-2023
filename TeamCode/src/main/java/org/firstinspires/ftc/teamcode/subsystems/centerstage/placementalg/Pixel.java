package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.PixelColor.*;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.ScoringMeasurements.*;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public final class Pixel implements Comparable<Pixel> {

    public int compareTo(Pixel cPixel) {
        double diff = cPixel.scoreValue - scoreValue;
        if (diff == 0) diff = y - cPixel.y;
        if (diff == 0) diff = x - cPixel.x;
        return (int) (diff * 1000000000);
    }

    final int x, y;
    Pixel mosaic = null;
    public final PixelColor color;
    double scoreValue = 0;

    Pixel(int x, int y, PixelColor color) {
        this.x = x;
        this.y = y;
        this.color = color;
    }

    Pixel(Pixel p, PixelColor color) {
        this(p.x, p.y, color);
        this.scoreValue = p.scoreValue;
    }

    Pixel(Pixel p) {
        this(p, p.color);
    }

    boolean inMosaic() {
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

    void print() {
        System.out.println(this);
    }
}
