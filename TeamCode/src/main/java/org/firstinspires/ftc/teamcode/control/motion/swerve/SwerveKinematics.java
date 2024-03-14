package org.firstinspires.ftc.teamcode.control.motion.swerve;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.max;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public final class SwerveKinematics {

    public static double
            WIDTH = 10,
            LENGTH = 10;

    public static SwervePodState[] robotToPodStates(Pose2d drive) {

        double

        iY = drive.getY(),
        iX = drive.getX(),
        t = drive.getHeading() / hypot(WIDTH, LENGTH),

        tY = t * LENGTH,
        tX = t * WIDTH,

        a = iY - tY,
        b = iY + tY,
        c = iX - tX,
        d = iX + tX,

        vBR = hypot(a, d),
        vBL = hypot(a, c),
        vFR = hypot(b, d),
        vFL = hypot(b, c),

        aBR = atan2(a, d),
        aBL = atan2(a, c),
        aFR = atan2(b, d),
        aFL = atan2(b, c),

        max = max(1.0, max(
                max(abs(vBR), abs(vBL)),
                max(abs(vFR), abs(vFL))
        ));

        return new SwervePodState[]{
            new SwervePodState(vBR / max, aBR),
            new SwervePodState(vBL / max, aBL),
            new SwervePodState(vFR / max, aFR),
            new SwervePodState(vFL / max, aFL),
        };
    }
}
