package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.randomizations;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRed;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRight;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.AutonPixelSupplier;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.subsystems.utilities.sensors.TeamPropDetector;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(preselectTeleOp = "MainTeleOp")
public final class MainAuton extends LinearOpMode {

    public static final double
            LEFT = PI,
            FORWARD = 1.5707963267948966,
            RIGHT = 0,
            BACKWARD = -1.5707963267948966;

    // Declare objects:
    public static GamepadEx gamepadEx1, gamepadEx2;
    public static MultipleTelemetry mTelemetry;
    static Robot robot;
    public static Backdrop autonBackdrop = new Backdrop();
    static Pose2d autonEndPose = new Pose2d(0, 0, FORWARD);

    public static boolean keyPressed(int gamepad, GamepadKeys.Button button) {
        return (gamepad == 2 ? gamepadEx2 : gamepadEx1).wasJustPressed(button);
    }

    public static double
            X_START_LEFT = -35,
            X_START_RIGHT = 12,
            X_AFTER_SPIKE = 24;

    public static EditablePose
            startPose = new EditablePose(X_START_RIGHT, -61.788975, FORWARD),
            centerSpike = new EditablePose(X_START_RIGHT, -26, FORWARD),
            leftSpike = new EditablePose(2.5, -36, toRadians(150)),
            rightSpike = new EditablePose(24 - leftSpike.x, leftSpike.y, LEFT - leftSpike.heading),
            parking = new EditablePose(Backdrop.X, -60, LEFT),
            parked = new EditablePose(60, -60, LEFT);

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry);

        // Initialize robot:
        robot = new Robot(hardwareMap);
        robot.preload();

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        boolean partnerWillDoRand = false;
        // Get gamepad 1 button input and save alliance and side for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (keyPressed(1, DPAD_RIGHT))  isRight = true;
            if (keyPressed(1, DPAD_LEFT))   isRight = false;
            if (keyPressed(1, B))           isRed = true;
            if (keyPressed(1, X))           isRed = false;
            if (keyPressed(1, Y))           partnerWillDoRand = !partnerWillDoRand;
            mTelemetry.addLine("Selected " + (isRed ? "RED" : "BLUE") + " " + (isRight ? "RIGHT" : "LEFT"));
            mTelemetry.addLine("Your alliance partner will " + (partnerWillDoRand ? "not " : "") + "be placing a yellow pixel");
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }
        mTelemetry.addLine("Confirmed " + (isRed ? "RED" : "BLUE") + " " + (isRight ? "RIGHT" : "LEFT"));
        mTelemetry.update();
        isRight = isRight == isRed;

        TrajectorySequence[] sequences = new TrajectorySequence[3];

        Pose2d startPose = MainAuton.startPose.byBoth().toPose2d();
        robot.drivetrain.setPoseEstimate(startPose);

        for (PropDetectPipeline.Randomization rand : randomizations) {
            ArrayList<Pixel> placements = AutonPixelSupplier.getPlacements(rand, partnerWillDoRand);
            if (partnerWillDoRand) placements.remove(0);
            PropDetectPipeline.Randomization spikePos = rand;
            if (!isRed) {

                if (rand == PropDetectPipeline.Randomization.RIGHT) spikePos = PropDetectPipeline.Randomization.LEFT;
                else if (rand == PropDetectPipeline.Randomization.LEFT) spikePos = PropDetectPipeline.Randomization.RIGHT;
            }

            Pose2d spike = (
                    spikePos == PropDetectPipeline.Randomization.LEFT ? leftSpike :
                            spikePos == PropDetectPipeline.Randomization.RIGHT ? rightSpike :
                                    centerSpike).byBoth().toPose2d();

            Pose2d afterSpike = new EditablePose(spike.getX() + X_AFTER_SPIKE, spike.getY(), LEFT).byAlliance().toPose2d();

            sequences[rand.ordinal()] = spikePos == PropDetectPipeline.Randomization.LEFT || spikePos == PropDetectPipeline.Randomization.RIGHT ?

                    robot.drivetrain.trajectorySequenceBuilder(startPose)
                    .setTangent(startPose.getHeading())
                    .forward(17)
                    .splineTo(spike.vec(), spike.getHeading())
                    .back(5)
//                    .setTangent(afterSpike.getHeading() + LEFT)
//                    .splineToLinearHeading(afterSpike, afterSpike.getHeading() + LEFT)
//                    .addTemporalMarker(() -> {
//                        robot.deposit.lift.setTargetRow(placements.get(0).y);
//                        robot.intake.setRequiredIntakingAmount(2);
//                    })
//                    .lineToSplineHeading(placements.get(0).toPose2d())
//                    .addTemporalMarker(() -> {
//                        robot.deposit.paintbrush.dropPixels(2);
//                        autonBackdrop.add(placements.get(0));
//                    })
//                    .waitSeconds(TIME_DROP_SECOND)
//                    .lineTo(parking.byAlliance().toPose2d().vec())
//                    .lineTo(parked.byAlliance().toPose2d().vec())
                    .build() :
                    robot.drivetrain.trajectorySequenceBuilder(startPose)
                            .setTangent(startPose.getHeading())
                            .splineTo(spike.vec(), spike.getHeading())
                            .back(5)
//                    .setTangent(afterSpike.getHeading() + LEFT)
//                    .splineToLinearHeading(afterSpike, afterSpike.getHeading() + LEFT)
//                    .addTemporalMarker(() -> {
//                        robot.deposit.lift.setTargetRow(placements.get(0).y);
//                        robot.intake.setRequiredIntakingAmount(2);
//                    })
//                    .lineToSplineHeading(placements.get(0).toPose2d())
//                    .addTemporalMarker(() -> {
//                        robot.deposit.paintbrush.dropPixels(2);
//                        autonBackdrop.add(placements.get(0));
//                    })
//                    .waitSeconds(TIME_DROP_SECOND)
//                    .lineTo(parking.byAlliance().toPose2d().vec())
//                    .lineTo(parked.byAlliance().toPose2d().vec())
                    .build()
            ;
        }

        robot.preload();

        PropDetectPipeline.Randomization location = PropDetectPipeline.Randomization.RIGHT;
        TeamPropDetector detector = new TeamPropDetector(
                hardwareMap,
                OpenCvCameraRotation.UPRIGHT,
                "camera front",
                telemetry
        );

        while (opModeInInit()) {
            detector.run();
            location = detector.getLocation();
            mTelemetry.addData("Location", detector.getLocation().name());
            mTelemetry.update();
        }

        detector.stop();
        robot.drivetrain.followTrajectorySequenceAsync(sequences[location.ordinal()]);

        // Control loop:
        while (opModeIsActive()) {
            // Manually clear old sensor data from the last loop:
            robot.readSensors();

            robot.drivetrain.update();
            robot.run();

            autonEndPose = robot.drivetrain.getPoseEstimate();
            // Push telemetry data
            robot.printTelemetry();
            mTelemetry.update();
        }
    }

}
