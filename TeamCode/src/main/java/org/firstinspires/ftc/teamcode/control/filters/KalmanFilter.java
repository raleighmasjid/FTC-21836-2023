package org.firstinspires.ftc.teamcode.control.filters;

import static java.lang.Math.max;

import org.firstinspires.ftc.teamcode.control.gainmatrices.KalmanGains;

public final class KalmanFilter extends com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter implements Filter {

    public KalmanFilter() {
        this(new KalmanGains());
    }

    public KalmanFilter(KalmanGains gains) {
        super(
                gains.sensorGain,
                gains.modelGain,
                max(gains.count, 2)
        );
    }

    public void setGains(KalmanGains gains) {
        Q = gains.sensorGain;
        R = gains.modelGain;
        solveDARE();
    }

    @Override
    public double calculate(double newValue) {
        return super.estimate(newValue);
    }

    @Override
    public void reset() {
        super.initializeStackWith0();
    }
}
