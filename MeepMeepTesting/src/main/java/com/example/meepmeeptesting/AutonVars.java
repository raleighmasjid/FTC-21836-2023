package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.MainAuton.FORWARD;
import static com.example.meepmeeptesting.MainAuton.LEFT;
import static java.lang.Math.PI;

public class AutonVars {

    public static boolean
            isRed = true,
            backdropSide = true,
            cycle = false,
            partnerWillDoRand = false;

    static final int[] ourPlacements = {1, 3, 6};

    public static double
            SIZE_WINDOW = 720,
            LENGTH_ROBOT = 17.3984665354,
            WIDTH_ROBOT = 16.4220472441,
            SIZE_HALF_FIELD = 70.5,
            SIZE_TILE = 23.625,
            X_START_LEFT = SIZE_TILE * -1.5,
            X_START_RIGHT = SIZE_TILE * 0.5,
            Y_START = -SIZE_HALF_FIELD + LENGTH_ROBOT * 0.5,
            X_SHIFT_CENTER_AUDIENCE_STACK_CLEARANCE = -14,
            X_INTAKING = -55,
            Y_INTAKING_1 = -11,
            Y_INTAKING_2 = -23.625,
            Y_INTAKING_3 = -35.4375,
            X_SHIFT_PRE_STACK_AUDIENCE_INNER_SPIKE = 6,
            Y_SHIFT_POST_INNER_SPIKE_BACKDROP = 1,
            TIME_SPIKE_BACKDROP = 0.75,
            TIME_PRE_SPIKE_AUDIENCE_PAINTBRUSH = 0.5,
            TIME_SPIKE_AUDIENCE = 1,
            TIME_SPIKE_TO_INTAKE_FLIP = 0.5,
            TIME_PRE_YELLOW = 0.5,
            TIME_INTAKING = 2,
            X_SHIFT_INTAKING = 2,
            SPEED_INTAKING = 1,
            SPEED_INTAKE_STACK_APPROACH = 0.1,
            BOTTOM_ROW_HEIGHT = 2.5,
            X_BACKDROP = 49.5,
            Y_BACKDROP_0_BLUE = 43,
            Y_BACKDROP_0_RED = -28.85,
            WIDTH_PIXEL = 3.15,
            ANGLE_OUTER_SPIKE_APPROACH_RED = 5,
            ANGLE_OUTER_SPIKE_APPROACH_BLUE = 7.5,
            ANGLE_INNER_SPIKE_AUDIENCE_APPROACH = 1.3,
            HEIGHT_SPIKE_AUDIENCE = 0;

    public static EditablePose
            startPose = new EditablePose(X_START_RIGHT, Y_START, FORWARD),
            centerSpikeBackdrop = new EditablePose(15, -22.5, LEFT),
            innerSpikeBackdrop = new EditablePose(5.4, -35, LEFT),
            outerSpikeBackdrop = new EditablePose(28, -32, LEFT),
            centerSpikeAudience = new EditablePose(-47, -12, 3 * PI / 4.0),
            innerSpikeAudience = new EditablePose(-38, -30, LEFT),
            outerSpikeAudience = new EditablePose(-47, -12, FORWARD),
            postOuterAudience = new EditablePose(-36, -SIZE_TILE * .5, LEFT),
            parking = new EditablePose(X_BACKDROP, -60, LEFT),
            parked = new EditablePose(60, parking.y, LEFT),
            enteringBackstage = new EditablePose(22, -12, LEFT),
            offsetAudienceInner = new EditablePose(0, 0, 0),
            offsetAudienceOuter = new EditablePose(0, 0, 0),
            offsetAudienceCenter = new EditablePose(0, 0, 0);
}
