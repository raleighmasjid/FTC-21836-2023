package com.example.meepmeeptesting;

public class PropDetectPipeline {

    public enum Randomization {
        LEFT,
        CENTER,
        RIGHT;

        public static final Randomization[] randomizations = values();
    }
}