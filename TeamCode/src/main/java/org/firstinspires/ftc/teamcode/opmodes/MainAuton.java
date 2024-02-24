package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.PropDetectPipeline.Randomization.randomizations;
import static org.firstinspires.ftc.teamcode.opmodes.AutonCycles.driveToStack1;
import static org.firstinspires.ftc.teamcode.opmodes.AutonCycles.intake2Pixels;
import static org.firstinspires.ftc.teamcode.opmodes.AutonCycles.score;
import static org.firstinspires.ftc.teamcode.opmodes.AutonPreloads.audiencePreloadsAndWhite;
import static org.firstinspires.ftc.teamcode.opmodes.AutonPreloads.backdropPreloads;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.WIDTH_PIXEL;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.X_BACKDROP;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.Y_BACKDROP_0_BLUE;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.Y_BACKDROP_0_RED;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.cycle;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.ourPlacements;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.parked;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.parking;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.backdropSide;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.partnerWillDoRand;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_PARK;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_PARTNER;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_SIDE;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.selections;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FIVE_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FOUR_STACK;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.isRed;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.AutonPixelSupplier.getOtherPlacement;
import static java.lang.Math.PI;
import static java.util.Collections.swap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.vision.detectors.TeamPropDetector;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.PropDetectPipeline;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.AutonPixelSupplier;

import java.util.ArrayList;

@Autonomous(preselectTeleOp = "MainTeleOp")
public final class MainAuton extends LinearOpMode {

    public static final double
            REVERSE = PI,
            LEFT = REVERSE,
            FORWARD = 1.5707963267948966,
            RIGHT = 0,
            BACKWARD = -1.5707963267948966;

    // Declare objects:
    public static GamepadEx gamepadEx1, gamepadEx2;
    public static MultipleTelemetry mTelemetry;
    static Robot robot;
    public static Backdrop autonBackdrop = new Backdrop();
    static Pose2d autonEndPose = null;

    public static boolean keyPressed(int gamepad, GamepadKeys.Button button) {
        return (gamepad == 2 ? gamepadEx2 : gamepadEx1).wasJustPressed(button);
    }

    /**
     * @return A {@link Pose2d} corresponding to the phsyical scoring location of this {@link Pixel}
     */
    public static Pose2d toPose2d(Pixel pixel) {
        return new Pose2d(
                X_BACKDROP,
                (isRed ? Y_BACKDROP_0_RED : Y_BACKDROP_0_BLUE)
                        - ((pixel.x - 1) * WIDTH_PIXEL)
                        - (pixel.y % 2 != 0 ? 0.5 * WIDTH_PIXEL : 0
                ),
                PI
        );
    }

    enum AutonConfig {
        EDITING_LEFT,
        EDITING_CENTER,
        EDITING_RIGHT,
        EDITING_ALLIANCE,
        EDITING_SIDE,
        EDITING_PARK,
        EDITING_PARTNER;

        public static final AutonConfig[] selections = values();

        public AutonConfig plus(int i) {
            return selections[loopMod(ordinal() + i, selections.length)];
        }
        public String markIf(AutonConfig s) {
            return this == s ? " <" : "";
        }
    }

    public static int loopMod(int a, int b) {
        return (int) loopMod(a,(double) b);
    }

    public static double loopMod(double a, double b) {
        return (a % b + b) % b;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry);

        // Initialize robot:
        robot = new Robot(hardwareMap);
        robot.initRun();

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        AutonConfig selection = EDITING_ALLIANCE;

        // Get gamepad 1 button input and save alliance and side for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();

            if (keyPressed(1, DPAD_UP))   selection = selection.plus(-1);
            if (keyPressed(1, DPAD_DOWN)) selection = selection.plus(1);

            if (keyPressed(1, X)) switch (selection) {
                case EDITING_LEFT:
                case EDITING_CENTER:
                case EDITING_RIGHT:
                    int index = selection.ordinal();
                    ourPlacements[index] = getOtherPlacement(ourPlacements[index]);
                    break;
                case EDITING_ALLIANCE:
                    isRed = !isRed;
                    break;
                case EDITING_SIDE:
                    backdropSide = !backdropSide;
                    break;
                case EDITING_PARK:
                    cycle = !cycle;
                    break;
                default:
                case EDITING_PARTNER:
                    partnerWillDoRand = !partnerWillDoRand;
                    break;
            }

            printConfig(selection);
            mTelemetry.addLine();
            mTelemetry.addLine();
            mTelemetry.addLine("Press both shoulder buttons to CONFIRM!");

            mTelemetry.update();
        }
        robot.preload();
        robot.initRun();

        TeamPropDetector detector = new TeamPropDetector(hardwareMap);

        Pose2d startPose = AutonVars.startPose.byBoth().toPose2d();
        robot.drivetrain.setPoseEstimate(startPose);

        TrajectorySequence[] sequences = generateTrajectories(startPose);

        while (opModeInInit()) {
            detector.printTelemetry();
            mTelemetry.addLine();
            mTelemetry.addLine();
            printConfig(selection);
            mTelemetry.update();
        }

        PropDetectPipeline.Randomization location = detector.pipeline.getLocation();
        detector.stop();

        robot.drivetrain.followTrajectorySequenceAsync(sequences[location.ordinal()]);

        // Control loop:
        while (opModeIsActive()) {
            // Manually clear old sensor data from the last loop:
            robot.readSensors();
            robot.run();

            autonEndPose = robot.drivetrain.getPoseEstimate();
            // Push telemetry data
            robot.printTelemetry();
            mTelemetry.update();
        }
    }

    private void printConfig(AutonConfig selection) {
        mTelemetry.addLine((isRed ? "RED " : "BLUE ") + selection.markIf(EDITING_ALLIANCE));
        mTelemetry.addLine();
        mTelemetry.addLine((backdropSide ? "BACKDROP " : "AUDIENCE ") + "side" + selection.markIf(EDITING_SIDE));
        mTelemetry.addLine();
        mTelemetry.addLine("WILL " + (!cycle ? "PARK" : "CYCLE") + selection.markIf(EDITING_PARK));
        mTelemetry.addLine();
        mTelemetry.addLine("PARTNER " + (partnerWillDoRand ? "PLACES" : "DOESN'T PLACE") + " YELLOW" + selection.markIf(EDITING_PARTNER));
        mTelemetry.addLine();
        mTelemetry.addLine("Randomizations:");
        for (int i = 0; i < ourPlacements.length; i++) mTelemetry.addLine(
                randomizations[i].name() + ": " +
                (ourPlacements[i] % 2 == 1 ? "left" : "right") +
                selection.markIf(selections[i])
        );
    }

    @NonNull
    private static TrajectorySequence[] generateTrajectories(Pose2d startPose) {

        TrajectorySequence[] sequences = new TrajectorySequence[3];

        for (PropDetectPipeline.Randomization rand : randomizations) {

            ArrayList<Pixel> placements = AutonPixelSupplier.getPlacements(partnerWillDoRand, ourPlacements[rand.ordinal()]);
            if (partnerWillDoRand) {
                autonBackdrop.add(placements.get(0));
                placements.remove(0);
            }
            if (!backdropSide) swap(placements, 0, 1);

            boolean outer, inner;
            switch (rand) {
                case LEFT:
                    outer = !(inner = (backdropSide == isRed));
                    break;
                case RIGHT:
                    inner = !(outer = (backdropSide == isRed));
                    break;
                default:
                    outer = inner = false;
                    break;
            }
            double a = isRed ? 1 : -1;

            TrajectorySequenceBuilder sequence = robot.drivetrain.trajectorySequenceBuilder(startPose)
                    .setTangent(startPose.getHeading())
                    ;

            if (backdropSide) backdropPreloads(sequence, placements, a, outer, inner);
            else audiencePreloadsAndWhite(sequence, placements, a, outer, inner);

            if (cycle) {

                Intake.Height height = backdropSide ? FIVE_STACK : FOUR_STACK;
                int placement = backdropSide ? 1 : 2;

                // CYCLE 1
                driveToStack1(sequence, height);
                intake2Pixels(sequence, height);
                score(sequence, placements, placement);

                // CYCLE 2
//                if (backdropSide) {
//                    driveToStack1(sequence, height.minus(2));
//                    intake2Pixels(sequence, 1, height.minus(2));
//                    score(sequence, placements, placement + 2);
//                }
            } else if (partnerWillDoRand) {
                sequence
                        .lineTo(parking.byAlliance().toPose2d().vec())
                        .lineTo(parked.byAlliance().toPose2d().vec())
                ;
            }

            sequences[rand.ordinal()] = sequence.build();
        }
        return sequences;
    }

}
