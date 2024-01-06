package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.autonBackdrop;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_FIRST;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_SECOND;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRed;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.PlacementCalculator.getOptimalPlacements;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.PlacementCalculator.getOptimalPlacementsWithExtraWhites;

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
    private volatile boolean pixelsJustTransferred = false, clearingScan = false, justScored = false;
    private volatile Color[] depositColors = {EMPTY, EMPTY};

    public BackdropScanner(Robot robot) {
        this.robot = robot;
        calculateColorsNeeded();
    }

    public void beginTrajectoryGeneration(Color[] depositColors) {
        this.depositColors = depositColors;
        pixelsJustTransferred = true;
    }

    public void clearScan() {
        clearingScan = true;
        latestScan.clear();
        calculateColorsNeeded();
        clearingScan = false;
    }

    /**
     * Runs the CV pipeline to detect backdrop changes. <br>
     * This method is run in an infinite loop in a separate thread, so
     * use volatile variables for any data used in this method. <br>
     * This method calls {@link #generateTrajectory()}
     */
    public void update() {
        Backdrop lastScan = latestScan.clone();

        // Detect (one of three) april tags on the (alliance-specific) backdrop (specified during pre-match config)

        // Skew image to fit april tag to square proportions

        // Shift image to some "standard configuration"

        // Check specific screen pixel coordinates to get colors

        // Save colors to corresponding locations in newScan

        if (justScored || !latestScan.equals(lastScan)) {
            justScored = false;
            timeSinceUpdate.reset();
            if (!clearingScan) calculateColorsNeeded();
        }

        if (pixelsJustTransferred) {
            pixelsJustTransferred = false;
            trajectoryReady = generateTrajectory();
        }
    }

    /**
     * Processes the colors from the deposit, matches them to backdrop placements,
     * and generates a trajectory to score them. <br>
     * This method is called from a parallel thread, so
     * use volatile variables for any data used in this method.
     *
     * @return Returns true if the trajectory has successfully been generated
     */
    private boolean generateTrajectory() {

        ArrayList<Pixel> optimalPlacementsCopy = new ArrayList<>(optimalPlacements);

        Color firstColor = depositColors[0], secondColor = depositColors[1];

        if (firstColor == EMPTY && secondColor == EMPTY) return false;

        placements[0] = new Pixel((isRed ? -2 : 9), 0, EMPTY);
        placements[1] = new Pixel((isRed ? -2 : 9), 0, EMPTY);

        if (firstColor != EMPTY) for (Pixel pixel : optimalPlacementsCopy) {
            if (firstColor.matches(pixel.color)) {

                Pixel placement = new Pixel(pixel, firstColor);

                placements[0] = placement;
                optimalPlacementsCopy = getOptimalPlacementsWithExtraWhites(latestScan.clone().add(placement));

                break;
            }
        }
        if (secondColor != EMPTY) for (Pixel pixel : optimalPlacementsCopy) {
            if (secondColor.matches(pixel.color)) {

                placements[1] = new Pixel(pixel, secondColor);
                break;
            }
        }

        Pose2d scoringPos1 = placements[0].toPose2d();
        Pose2d scoringPos2 = placements[1].toPose2d();

        Pose2d startPose = robot.drivetrain.getPoseEstimate();

        boolean sameScoringLocation = scoringPos1.epsilonEqualsHeading(scoringPos2);

        scoringTrajectory =
                firstColor == EMPTY || sameScoringLocation ?
                        robot.drivetrain.trajectorySequenceBuilder(startPose)
                                .addTemporalMarker(() -> {
                                    robot.deposit.lift.setTargetRow(placements[1].y);
                                })
                                .lineToSplineHeading(scoringPos2)
                                .addTemporalMarker(() -> {
                                    robot.deposit.paintbrush.dropPixels(2);
                                    latestScan.add(placements[1]);
                                    justScored = true;
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
                                    justScored = true;
                                })
                                .waitSeconds(TIME_DROP_SECOND)
                                .addTemporalMarker(() -> {
                                    trajectoryReady = false;
                                })
                                .build()
        ;
        return true;
    }

    private void calculateColorsNeeded() {
        optimalPlacements = getOptimalPlacementsWithExtraWhites(latestScan);

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
