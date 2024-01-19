package com.example.meepmeeptesting;

public class PropDetectPipeline {

    public enum Randomization {
        LEFT(1, 2),
        CENTER(3, 4),
        RIGHT(6, 5);

        Randomization(int x1, int x2) {
            this.x1 = x1;
            this.x2 = x2;
        }

        public final int x1, x2;
        public static final Randomization[] randomizations = values();
    }
}