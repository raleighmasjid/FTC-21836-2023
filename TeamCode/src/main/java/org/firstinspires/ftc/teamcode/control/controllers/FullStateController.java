package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.gainmatrices.FullStateGains;

public class FullStateController implements FeedbackController {

    private FullStateGains gains = new FullStateGains();
    private State target = new State();

    @Override
    public void setTarget(State target) {
        this.target = target;
    }

    public void setGains(FullStateGains gains) {
        this.gains = gains;
    }

    @Override
    public double calculate(State measurement) {
        State error = target.minus(measurement);

        return (error.x * gains.pGain) + (error.v * gains.vGain) + (error.a * gains.aGain);
    }
}
