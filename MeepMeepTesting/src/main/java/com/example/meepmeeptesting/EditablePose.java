package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.MeepMeepTesting.X_START_LEFT;
import static com.example.meepmeeptesting.MeepMeepTesting.X_START_RIGHT;
import static com.example.meepmeeptesting.MeepMeepTesting.backdropSide;
import static com.example.meepmeeptesting.MeepMeepTesting.isRed;

import com.acmerobotics.roadrunner.geometry.Pose2d;

class EditablePose {

    public double x, y, heading;

    public EditablePose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public EditablePose byAlliance() {
        double alliance = isRed ? 1 : -1;
        y *= alliance;
        heading *= alliance;
        return this;
    }

    public EditablePose bySide() {
        if (!backdropSide) x += X_START_LEFT - X_START_RIGHT;
        return this;
    }

    public EditablePose byBoth() {
        return byAlliance().bySide();
    }

    public Pose2d toPose2d() {
        return new Pose2d(x, y, heading);
    }

    public EditablePose flipBySide() {
        if (!backdropSide) heading = Math.PI - heading;
        if (!backdropSide) x = (X_START_LEFT + X_START_RIGHT) - x;
        return this;
    }
}