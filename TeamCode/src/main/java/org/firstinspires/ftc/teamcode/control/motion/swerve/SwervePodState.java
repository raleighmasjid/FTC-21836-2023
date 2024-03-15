package org.firstinspires.ftc.teamcode.control.motion.swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.PI;
import static java.lang.Math.abs;

public final class SwervePodState {

    public double velo, theta;

    public SwervePodState(double velo, double theta) {
        this.velo = velo;
        this.theta = normalizeRadians(theta);
    }

    public void optimize(SwervePodState real) {
        if (abs(real.theta - theta) <= 0.5 * PI) return;
        velo *= -1;
        theta = normalizeRadians(theta + PI);
    }
}
