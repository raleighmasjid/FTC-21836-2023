package com.example.meepmeeptesting;

import static java.lang.Math.asin;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;

public final class Intake {

    public static double
            ANGLE_PIVOT_OFFSET = 13,
            ANGLE_PIVOT_FLOOR_CLEARANCE = 5,
            ANGLE_PIVOT_TRANSFERRING = 197,
            ANGLE_PIVOT_CLIMBING = 50,
            ANGLE_LATCH_INTAKING = 105,
            ANGLE_LATCH_LOCKED = 159,
            ANGLE_LATCH_TRANSFERRING = 0,
            TIME_PIXEL_1_SETTLING = 0.5,
            TIME_PIXEL_2_SETTLING = 0,
            TIME_REVERSING = 1,
            TIME_PIVOTING = 0,
            TIME_SETTLING = 0.2,
            COLOR_SENSOR_GAIN = 1,
            SPEED_SLOW_REVERSING = -0.25,
            r = 9.5019488189,
            theta0 = -0.496183876745;

    enum State {
        HAS_0_PIXELS,
        PIXEL_1_SETTLING,
        PIXEL_2_SETTLING,
        HAS_1_PIXEL,
        PIVOTING,
        PIXELS_FALLING,
        PIXELS_SETTLING,
    }

    public enum Height {
        FLOOR,
        TWO_STACK,
        THREE_STACK,
        FOUR_STACK,
        FIVE_STACK;

        public final double deltaX, deltaTheta;

        private static final Height[] values = values();

        public static Height get(int ordinal) {
            return values[ordinal];
        }

        public Height minus(int less) {
            return values[max(ordinal() - 1, 0)];
        }

        Height() {
            if (ordinal() == 0) {
                deltaTheta = 0;
                deltaX = 0;
                return;
            }

            double deltaY = ordinal() * 0.5 - 0.1;

            double theta1 = asin((r * sin(theta0) + deltaY) / r);
            deltaTheta = toDegrees(theta1 - theta0);
            deltaX = r * cos(theta1) - r * cos(theta0);
        }
    }
}