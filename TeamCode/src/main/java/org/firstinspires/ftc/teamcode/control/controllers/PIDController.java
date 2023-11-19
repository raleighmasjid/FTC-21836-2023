package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.control.Differentiator;
import org.firstinspires.ftc.teamcode.control.Integrator;
import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.filters.Filter;
import org.firstinspires.ftc.teamcode.control.filters.NoFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;

public class PIDController implements FeedbackController {

    private PIDGains gains = new PIDGains();
    private State target = new State();

    public final Filter derivFilter;
    private final Differentiator differentiator = new Differentiator();
    private final Integrator integrator = new Integrator();

    private double error, errorIntegral, errorDerivative;

    public PIDController() {
        this(new NoFilter());
    }

    public PIDController(Filter derivFilter) {
        this.derivFilter = derivFilter;
    }

    public void setGains(PIDGains gains) {
        this.gains = gains;
    }

    /**
     * @param measurement Only the X attribute of the {@link State} parameter is used as feedback
     */
    public double calculate(State measurement) {
        double lastError = error;
        error = target.x - measurement.x;

        if (Math.signum(error) != Math.signum(lastError)) reset();
        errorIntegral = integrator.getIntegral(error);
        errorDerivative = derivFilter.calculate(differentiator.getDerivative(error));

        double output = (gains.kP * error) + (gains.kI * errorIntegral) + (gains.kD * errorDerivative);

        stopIntegration(Math.abs(output) >= gains.maxOutputWithIntegral && Math.signum(output) == Math.signum(error));

        return output;
    }

    public void setTarget(State target) {
        this.target = target;
    }

    public double getErrorDerivative() {
        return errorDerivative;
    }

    public double getErrorIntegral() {
        return errorIntegral;
    }

    public void stopIntegration(boolean stopIntegration) {
        integrator.stopIntegration(stopIntegration);
    }

    public void reset() {
        integrator.reset();
    }
}
