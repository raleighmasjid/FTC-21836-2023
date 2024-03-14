package org.firstinspires.ftc.teamcode.subsystems.drivetrains;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public interface Drivetrain {

    void run(double xCommand, double yCommand, double turnCommand, boolean useSlowMode);
    void lockSlowMode();

    double getHeading();
    void setCurrentHeading(double angle);

    TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose);
    void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence);
    void update();
    boolean isBusy();
    void breakFollowing();

    void printNumericalTelemetry();

    void setPoseEstimate(Pose2d pose);

    Pose2d getPoseEstimate();
}
