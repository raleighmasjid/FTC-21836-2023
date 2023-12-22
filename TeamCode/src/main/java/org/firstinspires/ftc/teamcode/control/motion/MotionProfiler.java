package org.firstinspires.ftc.teamcode.control.motion;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.gainmatrices.ProfileConstraints;

public final class MotionProfiler {

    private MotionProfile profile;

    private State state = new State();

    private final ElapsedTime profileTimer = new ElapsedTime();

    private ProfileConstraints constraints = new ProfileConstraints();

    public void setConstraints(ProfileConstraints constraints) {
        this.constraints = constraints;
    }

    public void generateProfile(State current, State target) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(current.x, current.v),
                new MotionState(target.x, target.v),
                constraints.maxV,
                constraints.maxA,
                constraints.maxJ
        );
        profileTimer.reset();
    }

    public void update() {
        MotionState mState = profile.get(profileTimer.seconds());
        this.state = new State(
                mState.getX(),
                mState.getV(),
                mState.getA(),
                mState.getJ()
        );
    }

    public double getX() {
        return state.x;
    }

    public double getV() {
        return state.v;
    }

    public double getA() {
        return state.a;
    }

    public double getJ() {
        return state.j;
    }

    public State getState() {
        return state;
    }

    public boolean isDone() {
        return profileTimer.seconds() >= profile.duration();
    }
}
