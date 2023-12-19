package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Backdrop.X;
import static com.example.meepmeeptesting.Backdrop.Y_MAX_BLUE;
import static com.example.meepmeeptesting.Backdrop.Y_MAX_RED;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public final class Pixel {

    public static double
            WIDTH = 3,
            HEIGHT = 2.59945;

    final int x;
    public final int y;

    Pixel(int x, int y) {
        this.x = x;
        this.y = y;
    }

    public Pose2d toPose2d(boolean isRed) {
        return new Pose2d(
                X,
                (isRed ? Y_MAX_RED : Y_MAX_BLUE) - (x * Pixel.WIDTH) + (y % 2 == 0 ? 0.5 * Pixel.WIDTH : 0),
                PI
        );
    }
}
