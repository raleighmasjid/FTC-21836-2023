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

        y = drive.getY(),
        x = drive.getX(),
        t = drive.getHeading() / hypot(WIDTH, LENGTH),

        yTurn = LENGTH * t,
        xTurn = WIDTH * t,

        a = -y + yTurn,
        b = -y - yTurn,
        c = x + xTurn,
        d = x - xTurn,

        br = hypot(a, d),
        bl = hypot(a, c),
        fr = hypot(b, d),
        fl = hypot(b, c),

        max = max(1.0, max(
                    max(abs(br), abs(bl)),
                    max(abs(fr), abs(fl))
        ));

        br /= max;
        bl /= max;
        fr /= max;
        fl /= max;

        return new SwervePodState[]{
            new SwervePodState(br, atan2(a, d)),
            new SwervePodState(bl, atan2(a, c)),
            new SwervePodState(fr, atan2(b, d)),
            new SwervePodState(fl, atan2(b, c)),
        };
    }
}
