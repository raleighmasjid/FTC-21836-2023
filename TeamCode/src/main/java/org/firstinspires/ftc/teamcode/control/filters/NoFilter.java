package org.firstinspires.ftc.teamcode.control.filters;

public class NoFilter implements Filter {

    private double lastValue;

    @Override
    public double calculate(double newValue) {
        lastValue = newValue;
        return lastValue;
    }

    @Override
    public void reset() {
        lastValue = 0;
    }
}
