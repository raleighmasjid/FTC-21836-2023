package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.RED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;
import org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator;

@Config
public final class Robot {

    public static double
            maxVoltage = 13,
            TIME_TRAJECTORY_GEN = 0;

    public static boolean isRed = true, isRight = true;

    public final MecanumDrivetrain drivetrain;
    public final Intake intake;
    public final Deposit deposit;

    private final BulkReader bulkReader;
    private final LEDIndicator[] indicators;

//    private final BackdropScanner scanner;

    private boolean autoDriveStarted = true;
    private final ElapsedTime autoTimer = new ElapsedTime();

    public Robot(HardwareMap hardwareMap) {
        bulkReader = new BulkReader(hardwareMap);

        drivetrain = new MecanumDrivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        deposit = new Deposit(hardwareMap);

//        scanner = new BackdropScanner(this);

        indicators = new LEDIndicator[]{
                new LEDIndicator(hardwareMap, "led right"),
                new LEDIndicator(hardwareMap, "led left")
        };
    }

    public void readSensors() {
        bulkReader.bulkRead();
    }

    public void startAutoDrive() {
//        TrajectorySequence scoringTrajectory = scanner.getScoringTrajectory();
//        if (scoringTrajectory == null) return;
//        drivetrain.followTrajectorySequenceAsync(scoringTrajectory);
        autoDriveStarted = false;
        autoTimer.reset();
    }

    public boolean beginUpdatingRunner() {
        if (!autoDriveStarted && autoTimer.seconds() >= TIME_TRAJECTORY_GEN) {
            autoDriveStarted = true;
            return true;
        } else return false;
    }

    private final ElapsedTime loopTimer = new ElapsedTime();

    public void run() {
        mTelemetry.addData("Before intake.pixelsTransferred()", loopTimer.seconds());
        if (intake.pixelsTransferred()) {
            deposit.paintbrush.lockPixels(intake.getColors());
//            scanner.generateTrajectory(deposit.paintbrush.getColors());
        }

        mTelemetry.addData("Before deposit.run()", loopTimer.seconds());
        deposit.run();
        mTelemetry.addData("Before intake.run()", loopTimer.seconds());
        intake.run(deposit.paintbrush.getPixelsLocked(), deposit.isRetracted());

        mTelemetry.addData("Before indicators", loopTimer.seconds());
        for (LEDIndicator indicator : indicators) {
            indicator.setState(drivetrain.isBusy() ? RED :
//                    scanner.trajectoryReady() ? GREEN :
                            OFF);
        }
        loopTimer.reset();
    }

    public void interrupt() {
        drivetrain.interrupt();
        intake.interrupt();
//        scanner.interrupt();
    }

    public void printTelemetry() {
//        scanner.printTelemetry();
        mTelemetry.addLine();
//        drivetrain.printTelemetry(mTelemetry);
//        mTelemetry.addLine();
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
