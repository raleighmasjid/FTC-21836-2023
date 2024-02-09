package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.YELLOW;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.RED;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.AutoScoringManager;
import org.firstinspires.ftc.teamcode.subsystems.drivetrains.AutoTurnMecanum;
import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;
import org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator;
import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;

@Config
public final class Robot {

    public static double
            maxVoltage = 13,
            TIME_TRAJECTORY_GEN = 0,
            ANGLE_DRONE_INITIAL = 50,
            ANGLE_DRONE_LAUNCHED = 0,
            ANGLE_SPIKE_LOCKED = 90,
            ANGLE_SPIKE_RELEASED = 0;

    public static boolean isRed = true;

    public final AutoTurnMecanum drivetrain;
    public final Intake intake;
    public final Deposit deposit;
    public final SimpleServoPivot drone, spike;

    private final BulkReader bulkReader;
    private final LEDIndicator[] indicators;

    public AutoScoringManager autoScoringManager = null;

    public Robot(HardwareMap hardwareMap) {
        bulkReader = new BulkReader(hardwareMap);

        drivetrain = new AutoTurnMecanum(hardwareMap);
        drivetrain.update();
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);
        drone = new SimpleServoPivot(
                ANGLE_DRONE_INITIAL,
                ANGLE_DRONE_LAUNCHED,
                getGoBildaServo(hardwareMap, "drone")
        );
        spike = new SimpleServoPivot(
                ANGLE_SPIKE_RELEASED,
                ANGLE_SPIKE_LOCKED,
                getGoBildaServo(hardwareMap, "floor pixel")
        );

        indicators = new LEDIndicator[]{
                new LEDIndicator(hardwareMap, "led left green", "led left red"),
                new LEDIndicator(hardwareMap, "led right green", "led right red")
        };
    }

    public void preload() {
        deposit.paintbrush.lockPixels(YELLOW);
        spike.setActivated(true);
    }

    public void initRun() {
        drone.run();
        spike.run();
    }

    public void startAlgorithm(HardwareMap hardwareMap) {
        autoScoringManager = new AutoScoringManager(hardwareMap, this);
    }

    public void readSensors() {
        bulkReader.bulkRead();
        intake.sensors[0].update();
        intake.sensors[1].update();
        drivetrain.update();
        deposit.lift.readSensors();
    }

    public boolean autoScore() {
        TrajectorySequence scoringTrajectory = autoScoringManager.getScoringTrajectory();
        if (scoringTrajectory == null) return false;
        drivetrain.followTrajectorySequenceAsync(scoringTrajectory);
        return true;
    }

    public void run() {
        drone.updateAngles(ANGLE_DRONE_INITIAL, ANGLE_DRONE_LAUNCHED);
        spike.updateAngles(ANGLE_SPIKE_RELEASED, ANGLE_SPIKE_LOCKED);

        if (intake.pixelsTransferred()) {
            deposit.paintbrush.lockPixels(intake.colors);
            if (autoScoringManager != null) autoScoringManager.beginTrajectoryGeneration(deposit.paintbrush.colors);
        }

        intake.run(
                deposit.paintbrush.getPixelsLocked(),
                deposit.isRetracted(),
                deposit.lift.isExtended()
        );
        deposit.run(intake.clearOfDeposit());
        drone.run();
        spike.run();

        LEDIndicator.State ledColor =
                drivetrain.isBusy() ? RED :
                autoScoringManager != null && autoScoringManager.trajectoryReady() ? GREEN :
                OFF;

        for (LEDIndicator indicator : indicators) indicator.setState(ledColor);
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
