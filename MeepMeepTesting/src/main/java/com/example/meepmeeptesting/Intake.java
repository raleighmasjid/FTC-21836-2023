package com.example.meepmeeptesting;

import static java.lang.Math.asin;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;

public final class Intake {

    public static double
            ANGLE_PIVOT_OFFSET = 9,
            ANGLE_PIVOT_FLOOR_CLEARANCE = 3,
            ANGLE_PIVOT_TRANSFERRING = 196.7,
            ANGLE_PIVOT_VERTICAL = 110,
            ANGLE_LATCH_INTAKING = 105,
            ANGLE_LATCH_LOCKED = 159,
            ANGLE_LATCH_TRANSFERRING = 0,
            TIME_PIXEL_1_SETTLING = 0.25,
            TIME_PIVOTING = 0,
            TIME_SETTLING = 0.2,
            TIME_INTAKE_FLIP_TO_LIFT = 0.2,
            TIME_REVERSING = 0.175,
            COLOR_SENSOR_GAIN = 1,
            HEIGHT_SHIFT = -0.1,
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

        private static final Intake.Height[] values = values();

        public Intake.Height minus(int less) {
            return values[max(ordinal() - less, 0)];
        }

        Height() {
            if (ordinal() == 0) {
                deltaTheta = 0;
                deltaX = 0;
                return;
            }

            double deltaY = ordinal() * 0.5 + HEIGHT_SHIFT;

            double theta1 = asin((r * sin(theta0) + deltaY) / r);
            deltaTheta = toDegrees(theta1 - theta0);
            deltaX = r * cos(theta1) - r * cos(theta0);
        }
    }
}