package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_FIRST;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_SECOND;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.PlacementCalculator.getOptimalPlacements;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;

import java.util.ArrayList;

public final class BackdropScanner extends Thread {

    public static double
            RUNNING_OFFSET_X = 10,
            RUNNING_OFFSET_Y = 10;

    private boolean run = true;
    private final ElapsedTime timeSinceUpdate = new ElapsedTime();

    private final Backdrop latestScan = new Backdrop();
    private ArrayList<Pixel> optimalPlacements = new ArrayList<>();

    private final Pixel[] placements = new Pixel[]{new Pixel(-2, 0, EMPTY), new Pixel(-2, 0, EMPTY)};
    private final Pixel.Color[] colorsNeeded = {EMPTY, EMPTY};
    private TrajectorySequence scoringTrajectory = null;

    private final Robot robot;
    private boolean isRed = true, pixelsTransferred = false;
    private Pixel.Color[] depositColors = {EMPTY, EMPTY};

    public BackdropScanner(Robot robot) {
        this.robot = robot;
        start();
    }

    public void generateTrajectory(boolean isRed, Pixel.Color[] depositColors) {
        this.isRed = isRed;
        this.depositColors = depositColors;
        pixelsTransferred = true;
    }

    public void run() {
        while (run) {
            Backdrop lastScan = latestScan.clone();

            // Detect (one of three) april tags on the (alliance-specific) backdrop (specified during pre-match config)

            // Skew image to fit april tag to square proportions

            // Shift image to some "standard configuration"

            // Check specific screen pixel coordinates to get colors

            // Save colors to corresponding locations in newScan

            if (!latestScan.equals(lastScan)) {
                timeSinceUpdate.reset();
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

            if (pixelsTransferred) {
                ArrayList<Pixel> optimalPlacementsCopy = new ArrayList<>(optimalPlacements);

                Pixel.Color firstColor = depositColors[0], secondColor = depositColors[1];

                placements[0] = new Pixel((isRed ? -2 : 9), 0, EMPTY);
                placements[1] = new Pixel((isRed ? -2 : 9), 0, EMPTY);

                if (firstColor != EMPTY) for (Pixel pixel : optimalPlacementsCopy) {
                    if (firstColor.matches(pixel.color)) {

                        Pixel placement = new Pixel(pixel, firstColor);

                        placements[0] = placement;
                        optimalPlacementsCopy = getOptimalPlacements(latestScan.clone().add(placement));
                        break;
                    }
                }
                if (secondColor != EMPTY) for (Pixel pixel : optimalPlacementsCopy) {
                    if (secondColor.matches(pixel.color)) {

                        Pixel placement = new Pixel(pixel, secondColor);

                        placements[1] = placement;
                        break;
                    }
                }

                double side = isRed ? -1 : 1;

                Pose2d scoringPos1 = placements[0].toPose2d(isRed);
                Pose2d scoringPos2 = placements[1].toPose2d(isRed);

                Pose2d currentPose = robot.drivetrain.getPoseEstimate();
                Pose2d startPose = new Pose2d(currentPose.getX() + RUNNING_OFFSET_X, currentPose.getY() + side * RUNNING_OFFSET_Y, currentPose.getHeading());

                scoringTrajectory = robot.drivetrain.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .addTemporalMarker(() -> robot.deposit.lift.setTargetRow(placements[0].y))
                        .splineTo(scoringPos1.vec(), scoringPos1.getHeading())
                        .addTemporalMarker(() -> {
                            robot.deposit.paintbrush.dropPixels(1);
                            latestScan.add(placements[0]);
                        })
                        .waitSeconds(TIME_DROP_FIRST)
                        .addTemporalMarker(() -> robot.deposit.lift.setTargetRow(placements[1].y))
                        .lineTo(scoringPos2.vec())
                        .addTemporalMarker(() -> {
                            robot.deposit.paintbrush.dropPixels(2);
                            latestScan.add(placements[1]);
                        })
                        .waitSeconds(TIME_DROP_SECOND)
                        .build()
                ;

                pixelsTransferred = false;
            }
        }
    }

    public boolean trajectoryGenerated() {
        return scoringTrajectory != null;
    }

    public TrajectorySequence getScoringTrajectory() {
        TrajectorySequence copy = scoringTrajectory;
        scoringTrajectory = null;
        return copy;
    }

    public void interrupt() {
        run = false;
    }

    public void printTelemetry(MultipleTelemetry mTelemetry) {
        mTelemetry.addLine("Colors needed in wing:");
        mTelemetry.addLine("First: " +  colorsNeeded[0].name());
        mTelemetry.addLine("Second: " + colorsNeeded[1].name());
        mTelemetry.addLine();
        mTelemetry.addLine("Place on backdrop:");
        mTelemetry.addLine("First: (" + placements[0].x + ", " + placements[0].y + ")");
        mTelemetry.addLine("Second: (" + placements[1].x + ", " + placements[1].y + ")");
        mTelemetry.addLine();
        mTelemetry.addLine(timeSinceUpdate.seconds() <= 1 ? "Backdrop just changed!" : "No changes right now");
    }
}
