package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.YELLOW;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.RED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.AutoScoringManager;
import org.firstinspires.ftc.teamcode.subsystems.drivetrains.AutoTurnMecanum;
import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;
import org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator;

@Config
public final class Robot {

    public static double
            maxVoltage = 13,
            TIME_TRAJECTORY_GEN = 0;

    public static boolean isRed = true, backdropSide = true;

    public final AutoTurnMecanum drivetrain;
    public final Intake intake;
    public final Deposit deposit;

    private final BulkReader bulkReader;
    private final LEDIndicator[] indicators;

    public AutoScoringManager autoScoringManager = null;

    private boolean autoDriveStarted = true;
    private final ElapsedTime autoTimer = new ElapsedTime();

    public Robot(HardwareMap hardwareMap) {
        bulkReader = new BulkReader(hardwareMap);

        drivetrain = new AutoTurnMecanum(hardwareMap);
        drivetrain.update();
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);

        indicators = new LEDIndicator[]{
                new LEDIndicator(hardwareMap, "led left green", "led left red"),
                new LEDIndicator(hardwareMap, "led right green", "led right red")
        };
    }

    public void preload() {
        intake.setRequiredIntakingAmount(0);
        deposit.paintbrush.lockPixels(YELLOW, EMPTY);
    }

    public void startAlgorithm() {
        autoScoringManager = new AutoScoringManager(this);
    }

    public void readSensors() {
        bulkReader.bulkRead();
        intake.topSensor.update();
        intake.bottomSensor.update();
        drivetrain.update();
    }

    public void startAutoDrive() {
        TrajectorySequence scoringTrajectory = autoScoringManager.getScoringTrajectory();
        if (scoringTrajectory == null) return;
        drivetrain.followTrajectorySequenceAsync(scoringTrajectory);
        autoDriveStarted = false;
        autoTimer.reset();
    }

    public boolean beginUpdatingRunner() {
        if (!autoDriveStarted && autoTimer.seconds() >= TIME_TRAJECTORY_GEN) {
            autoDriveStarted = true;
            return true;
        } else return false;
    }

    public void run() {
        if (intake.pixelsTransferred()) {
            deposit.paintbrush.lockPixels(intake.getColors());
            if (autoScoringManager != null) autoScoringManager.beginTrajectoryGeneration(deposit.paintbrush.getColors());
        }

        deposit.run();
        intake.run(deposit.paintbrush.getPixelsLocked(), deposit.isRetracted());

        for (LEDIndicator indicator : indicators) indicator.setState(
                drivetrain.isBusy() ? RED :
                autoScoringManager != null && autoScoringManager.trajectoryReady() ? GREEN :
                OFF
        );
    }

    public void printTelemetry() {
        if (autoScoringManager != null) {
            autoScoringManager.printTelemetry();
            mTelemetry.addLine();
        }
        drivetrain.printTelemetry();
        mTelemetry.addLine();
        deposit.paintbrush.printTelemetry();
        mTelemetry.addLine();
        deposit.lift.printTelemetry();
        mTelemetry.addLine();
        intake.printTelemetry();
        mTelemetry.addLine();
        mTelemetry.addLine();
        mTelemetry.addLine();
        drivetrain.printNumericalTelemetry();
        mTelemetry.addLine();
        deposit.lift.printNumericalTelemetry();
        mTelemetry.addLine();
        intake.printNumericalTelemetry();
    }
}
