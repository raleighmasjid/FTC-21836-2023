package org.firstinspires.ftc.teamcode.control.motion.swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class SwerveModule {

    public static double
            ANTI_COAX_EFFECT_GAIN = 0,
            OFFSET_BR = 0,
            OFFSET_BL = 0,
            OFFSET_FR = 0,
            OFFSET_FL = 0;

    private final double podRotOffset;
    private final SwervePodState current;
    private SwervePodState target;

    public SwerveModule(double podRotOffset) {
        this.podRotOffset = podRotOffset;
        target = current = new SwervePodState(0, podRotOffset);
    }

    public void readSensors() {
        current.theta = normalizeRadians( - podRotOffset);
    }

    public void setTarget(SwervePodState target) {
        this.target = target;
    }

    public void run() {

        target.optimize(current);

        // set motors

        if (target.velo == 0) return;

        // set angle controllers

        // run cr servos
        // include + COAX_EFFECT_CORRECTION_GAIN * target.velo
    }

}
