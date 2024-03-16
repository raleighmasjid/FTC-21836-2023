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
        return robotToPodStates(drive, true);
    }

    public static SwervePodState[] robotToPodStates(Pose2d drive, boolean normalize) {

        double

        iY = drive.getY(),
        iX = drive.getX(),
        t = drive.getHeading() / hypot(WIDTH, LENGTH),

        // Calculate rotation component vectors
        tY = t * LENGTH,
        tX = t * WIDTH,

        // Combine component vectors into total x and y vectors (per pod)
        a = iY - tY,
        b = iY + tY,
        c = iX - tX,
        d = iX + tX,

        // Get velocity vector (hypotenuse) of total x and y vectors (per pod)
        vBR = hypot(a, d),
        vBL = hypot(a, c),
        vFR = hypot(b, d),
        vFL = hypot(b, c),

        // Calculate pod angles with total x and y vectors (per pod)
        aBR = atan2(a, d),
        aBL = atan2(a, c),
        aFR = atan2(b, d),
        aFL = atan2(b, c);

        if (normalize) {
            // Get max to normalize wheel velocities
            double max = max(1.0, max(
                    max(abs(vBR), abs(vBL)),
                    max(abs(vFR), abs(vFL))
            ));

            // Normalize motor powers to [-1, 1]
            vBR /= max;
            vBL /= max;
            vFR /= max;
            vFL /= max;
        }

        return new SwervePodState[]{
            new SwervePodState(vBR, aBR),
            new SwervePodState(vBL, aBL),
            new SwervePodState(vFR, aFR),
            new SwervePodState(vFL, aFL),
        };
    }
}
