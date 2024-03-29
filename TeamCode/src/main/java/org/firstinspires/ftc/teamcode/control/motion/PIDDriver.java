package org.firstinspires.ftc.teamcode.control.motion;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.opmodes.EditablePose;
import org.firstinspires.ftc.teamcode.subsystems.drivetrains.Drivetrain;

@Config
public final class PIDDriver {

    public static PIDGains
            xyGains = new PIDGains(
                    0,
                    0,
                    0,
                    1
            ),
            rotGains = new PIDGains(
                    0,
                    0,
                    0,
                    1
            );

    public static double STRAFE_MULTIPLIER = 1;

    public static EditablePose admissibleError = new EditablePose(0.01, 0.01, 0.001);

    private final PIDController
            xController = new PIDController(),
            yController = new PIDController(),
            rotController = new PIDController();

    public boolean driveTo(Drivetrain dt, Pose2d target) {

        Pose2d current = dt.getPoseEstimate();

        double currentX = current.getX();
        double currentY = current.getY();
        double currentHeading = current.getHeading();

        double targetX = target.getX();
        double targetY = target.getY();
        double targetHeading = target.getHeading();

        double xError = targetX - currentX;
        double yError = targetY - currentY;
        double headingError = normalizeRadians(targetHeading - currentHeading);

        if (
                abs(xError) <= admissibleError.x &&
                abs(yError) <= admissibleError.y &&
                abs(headingError) <= admissibleError.heading
        ) return true;

        xController.setGains(xyGains);
        yController.setGains(xyGains);
        rotController.setGains(rotGains);

        xController.setTarget(new State(targetX));
        yController.setTarget(new State(targetY*1));
        rotController.setTarget(new State(headingError + currentHeading));

        double x = xController.calculate(new State(currentX));
        double y = yController.calculate(new State(currentY*1));
        double rot = rotController.calculate(new State(currentHeading));

        double theta = -currentHeading;
        double cos = cos(theta);
        double sin = sin(theta);
        double x2 = x * cos - y * sin;
        double y2 = y * cos + x * sin;

        dt.setDrivePower(new Pose2d(
                x2,
                y2 * STRAFE_MULTIPLIER,
                rot
        ));

        return false;
    }
}
