package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.ParkingLocation.INNER;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.FORWARD;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.LEFT;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.loopMod;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutonVars {

    enum ParkingLocation {
        OUTER,
        CENTER,
        INNER;

        public static final ParkingLocation[] locations = values();

        public ParkingLocation plus(int i) {
            return locations[loopMod(ordinal() + i, locations.length)];
        }
    }


    public static boolean isRed = true;

    static boolean
            isBackdropSide = true,
            cycle = false,
            partnerWillDoRand = false;

    static final int[] ourPlacements = {1, 3, 6};

    static ParkingLocation parking = INNER;

    public static double
            SIZE_WINDOW = 720,
            LENGTH_ROBOT = 17.3984665354,
            WIDTH_ROBOT = 16.4220472441,
            SIZE_HALF_FIELD = 70.5,
            SIZE_TILE = 23.625,
            X_START_LEFT = SIZE_TILE * -1.5,
            X_START_RIGHT = SIZE_TILE * 0.5,
            Y_START = -SIZE_HALF_FIELD + LENGTH_ROBOT * 0.5,
            X_INTAKING = -52,
            Y_INTAKING_1 = -10,
            Y_INTAKING_2 = -23.625,
            Y_INTAKING_3 = -35.4375,
            X_SHIFT_PRE_STACK_AUDIENCE_INNER_SPIKE = 6,
            Y_SHIFT_POST_INNER_SPIKE_BACKDROP = 2,
            TIME_SPIKE_BACKDROP = 0.5,
            TIME_PRE_SPIKE_AUDIENCE_PAINTBRUSH = 0.5,
            TIME_SPIKE_AUDIENCE = 1,
            TIME_SPIKE_TO_INTAKE_FLIP = 0.5,
            TIME_PRE_YELLOW = 0.5,
            TIME_INTAKING = 2,
            X_SHIFT_INTAKING = 2,
            SPEED_INTAKING = 1,
            SPEED_INTAKE_STACK_APPROACH = 0.1,
            BOTTOM_ROW_HEIGHT = 2.5,
            X_BACKDROP = 51,
            Y_BACKDROP_0_BLUE = 43,
            Y_BACKDROP_0_RED = -29,
            WIDTH_PIXEL = 3,
            ANGLE_INNER_SPIKE_AUDIENCE_APPROACH = 1,
            HEIGHT_SPIKE_AUDIENCE = -0.5;

    public static EditablePose
            startPose = new EditablePose(X_START_RIGHT, Y_START, FORWARD),
            centerSpikeBackdrop = new EditablePose(19, -22.5, LEFT),
            innerSpikeBackdrop = new EditablePose(5.4, -35, LEFT),
            outerSpikeBackdrop = new EditablePose(30, -32, LEFT),
            centerSpikeAudience = new EditablePose(-47, -12, 3 * PI / 4.0),
            innerSpikeAudience = new EditablePose(-36, -30, LEFT),
            outerSpikeAudience = new EditablePose(-47, -12, FORWARD),
            parkingInner = new EditablePose(X_BACKDROP, -60, LEFT),
            parkedInner = new EditablePose(60, parkingInner.y, LEFT),
            parkingOuter = new EditablePose(X_BACKDROP, -12, LEFT),
            parkedOuter = new EditablePose(60, parkingOuter.y, LEFT),
            enteringBackstage = new EditablePose(22, -11, LEFT),
            offsetAudienceInner = new EditablePose(0, 0, 0),
            offsetAudienceOuter = new EditablePose(0, 0, 0),
            offsetAudienceCenter = new EditablePose(0, 0, 0);
}
