package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.PropDetectPipeline.Randomization.randomizations;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_PARK;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_PARTNER;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.EDITING_SIDE;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.AutonConfig.selections;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.EditablePose.backdropSide;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_FIRST;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_SECOND;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FIVE_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FOUR_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRed;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.AutonPixelSupplier.getOtherPlacement;
import static java.lang.Math.PI;
import static java.util.Collections.swap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.vision.pipelines.PropDetectPipeline;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.AutonPixelSupplier;
import org.firstinspires.ftc.teamcode.subsystems.utilities.sensors.TeamPropDetector;

import java.util.ArrayList;

@Config
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
                (isRed ? Y_BACKDROP_0_RED : Y_BACKDROP_0_BLUE) - ((pixel.x - 1) * MainAuton.WIDTH_PIXEL) - (pixel.y % 2 != 0 ? 0.5 * MainAuton.WIDTH_PIXEL : 0),
                PI
        );
    }

    public static double
            LENGTH_ROBOT = 17.3984665354,
            WIDTH_ROBOT = 16.4220472441,
            SIZE_HALF_FIELD = 72,
            SIZE_TILE = SIZE_HALF_FIELD / 3.0,
            X_START_LEFT = SIZE_TILE * -1.5,
            X_START_RIGHT = SIZE_TILE * 0.5,
            X_SHIFT_BACKDROP_AFTER_SPIKE = 8,
            Y_SHIFT_BEFORE_SPIKE = 15,
            Y_SHIFT_AFTER_SPIKE = 26,
            Y_SHIFT_AUDIENCE_AFTER_SPIKE = 16,
            X_SHIFT_CENTER_AUDIENCE_AFTER_SPIKE = -22,
            X_SHIFT_CENTER_AUDIENCE_STACK_CLEARANCE = -14,
            X_TILE = 24,
            X_INTAKING = -56,
            Y_INTAKING_1 = -12,
            Y_INTAKING_3 = -36,
            TIME_SPIKE = 0.75,
            TIME_SPIKE_TO_INTAKE_FLIP = 0.5,
            TIME_PRE_YELLOW = 0.5,
            X_SHIFT_INTAKING = 5,
            SPEED_INTAKING = 0.5,
            BOTTOM_ROW_HEIGHT = 2,
            X_BACKDROP = 51.5,
            Y_BACKDROP_0_BLUE = 44.75,
            Y_BACKDROP_0_RED = -27.5,
            WIDTH_PIXEL = 3.15,
            ANGLE_AWAY_TRUSS_SPIKE_APPROACH_RED = 5,
            ANGLE_AWAY_TRUSS_SPIKE_APPROACH_BLUE = 7.5;

    public static EditablePose
            startPose = new EditablePose(X_START_RIGHT, LENGTH_ROBOT * 0.5 - SIZE_HALF_FIELD, FORWARD),
            centerSpikeBackdrop = new EditablePose(15, -25.5, LEFT),
            innerSpikeBackdrop = new EditablePose(5.4, -35, LEFT),
            outerSpikeBackdrop = new EditablePose(28, -32, LEFT),
            parking = new EditablePose(X_BACKDROP, -60, LEFT),
            parked = new EditablePose(60, parking.y, LEFT),
            enteringBackstage = new EditablePose(36, -12, LEFT),
            movingToStack2 = new EditablePose(-45, -24, LEFT);

    private static Pose2d stackPos(int stack) {
        return new EditablePose(X_INTAKING, stack == 3 ? Y_INTAKING_3 : stack == 2 ? movingToStack2.y : Y_INTAKING_1, LEFT).byAlliance().toPose2d();
    }

    private static void driveToStack1(TrajectorySequenceBuilder sequence, Intake.Height height) {
        sequence
                .addTemporalMarker(() -> {
                    robot.intake.setHeight(height);
                })
                .setTangent(MainAuton.startPose.byAlliance().heading)
                .lineTo(MainAuton.enteringBackstage.byAlliance().toPose2d().vec())
                .setTangent(LEFT)
                .addTemporalMarker(() -> {
                })
                .splineTo(stackPos(1).vec(), LEFT)
        ;
    }

    private static void driveToStack2(TrajectorySequenceBuilder sequence, Intake.Height height) {
        Pose2d turnToStack1 = new EditablePose(X_START_LEFT + X_SHIFT_CENTER_AUDIENCE_STACK_CLEARANCE, Y_INTAKING_1, LEFT).byAlliance().toPose2d();
        sequence
                .addTemporalMarker(() -> {
                    robot.intake.setHeight(height);
                })
                .setTangent(MainAuton.startPose.byAlliance().heading)
                .splineToConstantHeading(MainAuton.enteringBackstage.byAlliance().toPose2d().vec(), LEFT)
                .splineTo(turnToStack1.vec(), LEFT)
                .lineTo(movingToStack2.byAlliance().toPose2d().vec())
                .lineTo(stackPos(2).vec())
        ;
    }

    private static void intake2Pixels(TrajectorySequenceBuilder sequence, int stack, Intake.Height height) {
        Intake.Height height2 = height.minus(1);
        sequence
                .addTemporalMarker(() -> {
                    robot.intake.setMotorPower(SPEED_INTAKING);
                    while (robot.intake.colors[0] == Pixel.Color.EMPTY) {Thread.yield();}
                    robot.intake.setMotorPower(0);
                })
                .back(X_SHIFT_INTAKING)
                .addTemporalMarker(() -> {
                    robot.intake.setHeight(height2);
                })
                .lineTo(stackPos(stack).vec())
                .addTemporalMarker(() -> {
                    robot.intake.setMotorPower(SPEED_INTAKING);
                    while (robot.intake.colors[1] == Pixel.Color.EMPTY) {Thread.yield();}
                    robot.intake.setMotorPower(0);
                })
        ;
    }

    private static void score(TrajectorySequenceBuilder sequence, ArrayList<Pixel> placements, int index) {
        Pixel first = placements.get(index);
        Pixel second = placements.get(index + 1);
        sequence
                .lineTo(MainAuton.enteringBackstage.byAlliance().toPose2d().vec())

                .addTemporalMarker(() -> {
                    robot.deposit.lift.setTargetRow(first.y);
                })
                .splineToConstantHeading(toPose2d(first).vec(), MainAuton.startPose.byAlliance().heading + REVERSE)
                .addTemporalMarker(() -> {
                    robot.deposit.paintbrush.dropPixel();
                    autonBackdrop.add(first);
                })
                .waitSeconds(TIME_DROP_FIRST)

                .addTemporalMarker(() -> {
                    robot.deposit.lift.setTargetRow(second.y);
                })
                .lineToConstantHeading(toPose2d(second).vec())
                .addTemporalMarker(() -> {
                    robot.deposit.paintbrush.dropPixel();
                    autonBackdrop.add(second);
                })
                .waitSeconds(TIME_DROP_SECOND)
        ;
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
        robot.preload();
        robot.initRun();

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        AutonConfig selection = EDITING_ALLIANCE;

        int[] ourPlacements = {1, 3, 6};
        boolean partnerWillDoRand = false, park = true;
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
                    park = !park;
                    break;
                default:
                case EDITING_PARTNER:
                    partnerWillDoRand = !partnerWillDoRand;
                    break;
            }

            mTelemetry.addLine((isRed ? "RED " : "BLUE ") + selection.markIf(EDITING_ALLIANCE));
            mTelemetry.addLine();
            mTelemetry.addLine((backdropSide ? "BACKDROP " : "AUDIENCE ") + "side" + selection.markIf(EDITING_SIDE));
            mTelemetry.addLine();
            mTelemetry.addLine("WILL " + (park ? "PARK" : "CYCLE") + selection.markIf(EDITING_PARK));
            mTelemetry.addLine();
            mTelemetry.addLine("PARTNER " + (partnerWillDoRand ? "PLACES" : "DOESN'T PLACE") + " YELLOW" + selection.markIf(EDITING_PARTNER));
            mTelemetry.addLine();
            mTelemetry.addLine("Randomizations:");
            for (int i = 0; i < ourPlacements.length; i++) mTelemetry.addLine(
                    randomizations[i].name() + ": " +
                    (ourPlacements[i] % 2 == 1 ? "left" : "right") +
                    selection.markIf(selections[i])
            );
            mTelemetry.addLine();
            mTelemetry.addLine();
            mTelemetry.addLine("Press both shoulder buttons to CONFIRM!");

            mTelemetry.update();
        }

        TrajectorySequence[] sequences = generateTrajectories(partnerWillDoRand, park, ourPlacements);

        TeamPropDetector detector = new TeamPropDetector(hardwareMap);

        while (opModeInInit()) {
            robot.initRun();

            mTelemetry.addData("Location", detector.run().name());
            mTelemetry.update();
        }

        robot.drivetrain.followTrajectorySequenceAsync(sequences[detector.getLocation().ordinal()]);

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

    @NonNull
    private static TrajectorySequence[] generateTrajectories(boolean partnerWillDoRand, boolean park, int[] ourPlacements) {

        Pose2d startPose = MainAuton.startPose.byBoth().toPose2d();
        robot.drivetrain.setPoseEstimate(startPose);

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
                    outer = !(inner = (backdropSide ? isRed : !isRed));
                    break;
                case RIGHT:
                    inner = !(outer = (backdropSide ? isRed : !isRed));
                    break;
                default:
                    outer = inner = false;
                    break;
            }

            TrajectorySequenceBuilder sequence = robot.drivetrain.trajectorySequenceBuilder(startPose)
                    .setTangent(startPose.getHeading())
            ;

            if (backdropSide) {

                if (inner) {
                    Pose2d spike = innerSpikeBackdrop.byAlliance().flipBySide().toPose2d();
                    sequence.splineTo(spike.vec(), spike.getHeading());
                } else {
                    sequence.lineToSplineHeading((
                            outer ?
                                    outerSpikeBackdrop.byAlliance().flipBySide() :
                                    centerSpikeBackdrop.byBoth()
                    ).toPose2d());
                }

                sequence
                        .addTemporalMarker(() -> {
                            robot.spike.toggle();
                        })
                        .waitSeconds(TIME_SPIKE)
                        .setTangent(RIGHT)
                        .UNSTABLE_addTemporalMarkerOffset(TIME_SPIKE_TO_INTAKE_FLIP, () -> {
                            robot.deposit.lift.setTargetRow(placements.get(0).y);
                        })
                        .splineToConstantHeading(toPose2d(placements.get(0)).vec(),
                                outer ?
                                        isRed ? ANGLE_AWAY_TRUSS_SPIKE_APPROACH_RED : ANGLE_AWAY_TRUSS_SPIKE_APPROACH_BLUE :
                                        RIGHT
                        )
                        .waitSeconds(TIME_PRE_YELLOW)
                        .addTemporalMarker(() -> {
                            robot.deposit.paintbrush.dropPixel();
                            autonBackdrop.add(placements.get(0));
                        })
                        .waitSeconds(TIME_DROP_SECOND)
                ;

            } else {

                if (inner) {
                    Pose2d spike = innerSpikeBackdrop.byAlliance().flipBySide().toPose2d();
                    sequence.splineTo(spike.vec(), spike.getHeading());
                } else {
                    sequence.lineToSplineHeading((
                            outer ?
                                    outerSpikeBackdrop.byAlliance().flipBySide() :
                                    centerSpikeBackdrop.byAlliance().flipBySide()
                    ).toPose2d());
                }



            }

            if (!park) {

                Intake.Height height = backdropSide ? FIVE_STACK : FOUR_STACK;
                int placement = backdropSide ? 1 : 2;

                // CYCLE 1
                driveToStack1(sequence, height);
//                intake2Pixels(sequence, 1, height);
//                score(sequence, placements, placement);

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

    public static class EditablePose {

        public double x, y, heading;

        static boolean backdropSide = true;

        public EditablePose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        public EditablePose byAlliance() {
            double alliance = isRed ? 1 : -1;
            return new EditablePose(
                    x,
                    y * alliance,
                    heading * alliance
            );
        }

        public EditablePose bySide() {
            return new EditablePose(
                    x + (backdropSide ? 0 : X_START_LEFT - X_START_RIGHT),
                    y,
                    heading
            );
        }

        public EditablePose flipBySide() {
            return new EditablePose(
                    backdropSide ? x : (X_START_LEFT + X_START_RIGHT) - x,
                    y,
                    backdropSide ? heading : Math.PI - heading
            );
        }

        public EditablePose byBoth() {
            return byAlliance().bySide();
        }

        public Pose2d toPose2d() {
            return new Pose2d(x, y, heading);
        }
    }
}
