package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.autonBackdrop;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_FIRST;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_SECOND;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRed;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.*;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.PlacementCalculator.getOptimalPlacements;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;

import java.util.ArrayList;

public final class BackdropScanner {

    private final ElapsedTime timeSinceUpdate = new ElapsedTime();

    private final Backdrop latestScan = autonBackdrop;
    private volatile ArrayList<Pixel> optimalPlacements = getOptimalPlacements(latestScan);

    private final Pixel[] placements = new Pixel[]{new Pixel(-2, 0, EMPTY), new Pixel(-2, 0, EMPTY)};
    private final Color[] colorsNeeded = {EMPTY, EMPTY};
    private volatile TrajectorySequence scoringTrajectory = null;
    private volatile boolean trajectoryReady = false;

    private final Robot robot;
    private volatile boolean pixelsTransferred = false, clearingScan = false;
    private Color[] depositColors = {EMPTY, EMPTY};

    public BackdropScanner(Robot robot) {
        this.robot = robot;
        calculateColorsNeeded();
    }

    public void generateTrajectory(Color[] depositColors) {
        this.depositColors = depositColors;
        pixelsTransferred = true;
    }

    public void clearScan() {
        clearingScan = true;
        latestScan.clear();
        calculateColorsNeeded();
        clearingScan = false;
    }

    public void update() {
        Backdrop lastScan = latestScan.clone();

        // Detect (one of three) april tags on the (alliance-specific) backdrop (specified during pre-match config)

        // Skew image to fit april tag to square proportions

        // Shift image to some "standard configuration"

        // Check specific screen pixel coordinates to get colors

        // Save colors to corresponding locations in newScan

        if (!latestScan.equals(lastScan)) {
            timeSinceUpdate.reset();
            if (!(robot.drivetrain.isBusy() || clearingScan)) calculateColorsNeeded();
        }

        if (pixelsTransferred) {
            ArrayList<Pixel> optimalPlacementsCopy = new ArrayList<>(optimalPlacements);

            Color firstColor = depositColors[0], secondColor = depositColors[1];

            placements[0] = new Pixel((isRed ? -2 : 9), 0, EMPTY);
            placements[1] = new Pixel((isRed ? -2 : 9), 0, EMPTY);

            boolean firstEmpty = firstColor == EMPTY;
            boolean secondEmpty = secondColor == EMPTY;

            if (!firstEmpty) for (Pixel pixel : optimalPlacementsCopy) {
                if (firstColor.matches(pixel.color)) {

                    Pixel placement = new Pixel(pixel, firstColor);

                    placements[0] = placement;
                    optimalPlacementsCopy = getOptimalPlacements(latestScan.clone().add(placement));
                    break;
                }
            }
            if (!secondEmpty) for (Pixel pixel : optimalPlacementsCopy) {
                if (secondColor.matches(pixel.color)) {

                    Pixel placement = new Pixel(pixel, secondColor);

                    placements[1] = placement;
                    break;
                }
            }

            Pose2d scoringPos1 = placements[0].toPose2d();
            Pose2d scoringPos2 = placements[1].toPose2d();

            Pose2d startPose = robot.drivetrain.getPoseEstimate();

            boolean sameScoringLocation = scoringPos1.epsilonEquals(scoringPos2);

            scoringTrajectory =
                    firstEmpty && secondEmpty ?
                            null :
                            firstEmpty || sameScoringLocation ?
                                    robot.drivetrain.trajectorySequenceBuilder(startPose)
                                            .addTemporalMarker(() -> {
                                                robot.deposit.lift.setTargetRow(placements[1].y);
                                            })
                                            .lineToSplineHeading(scoringPos2)
                                            .addTemporalMarker(() -> {
                                                robot.deposit.paintbrush.dropPixels(2);
                                                latestScan.add(placements[1]);
                                                calculateColorsNeeded();
                                            })
                                            .waitSeconds(TIME_DROP_SECOND)
                                            .addTemporalMarker(() -> {
                                                trajectoryReady = false;
                                            })
                                            .build() :
                                    robot.drivetrain.trajectorySequenceBuilder(startPose)
                                            .addTemporalMarker(() -> {
                                                robot.deposit.lift.setTargetRow(placements[0].y);
                                            })
                                            .lineToSplineHeading(scoringPos1)
                                            .addTemporalMarker(() -> {
                                                robot.deposit.paintbrush.dropPixels(1);
                                                latestScan.add(placements[0]);
                                            })
                                            .waitSeconds(TIME_DROP_FIRST)
                                            .addTemporalMarker(() -> {
                                                robot.deposit.lift.setTargetRow(placements[1].y);
                                            })
                                            .lineToConstantHeading(scoringPos2.vec())
                                            .addTemporalMarker(() -> {
                                                robot.deposit.paintbrush.dropPixels(2);
                                                latestScan.add(placements[1]);
                                                calculateColorsNeeded();
                                            })
                                            .waitSeconds(TIME_DROP_SECOND)
                                            .addTemporalMarker(() -> {
                                                trajectoryReady = false;
                                            })
                                            .build()
            ;

            if (scoringTrajectory != null) trajectoryReady = true;

            pixelsTransferred = false;
        }
    }

    private void calculateColorsNeeded() {
        optimalPlacements = getOptimalPlacements(latestScan);
        colorsNeeded[0] = EMPTY;
        colorsNeeded[1] = EMPTY;
        if (!optimalPlacements.isEmpty()) {
            Pixel optimalPlacement = optimalPlacements.get(0);

            colorsNeeded[0] = optimalPlacement.color;

            ArrayList<Pixel> futureOptimalPlacements = getOptimalPlacements(latestScan.clone().add(optimalPlacement));
            if (!futureOptimalPlacements.isEmpty()) {
                colorsNeeded[1] = futureOptimalPlacements.get(0).color;
            }
        }
    }

    public boolean trajectoryReady() {
        return trajectoryReady;
    }

    public TrajectorySequence getScoringTrajectory() {
        return trajectoryReady ? scoringTrajectory : null;
    }

    public void printTelemetry() {
        mTelemetry.addLine("Colors needed in wing:");
        mTelemetry.addLine("First: " + colorsNeeded[0].name());
        mTelemetry.addLine("Second: " + colorsNeeded[1].name());
        mTelemetry.addLine();
        mTelemetry.addLine("Place on backdrop:");
        mTelemetry.addLine("First: (" + placements[0].x + ", " + placements[0].y + ")");
        mTelemetry.addLine("Second: (" + placements[1].x + ", " + placements[1].y + ")");
        mTelemetry.addLine();
        mTelemetry.addLine(timeSinceUpdate.seconds() <= 1 ? "Backdrop just changed!" : "No changes right now");
        latestScan.toTelemetry();
    }
}
