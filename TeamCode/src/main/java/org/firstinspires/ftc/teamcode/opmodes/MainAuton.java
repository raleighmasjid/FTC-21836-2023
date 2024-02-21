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
import org.firstinspires.ftc.teamcode.subsystems.centerstage.TeamPropDetector;

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

    private static final int[] ourPlacements = {1, 3, 6};
    private static boolean partnerWillDoRand = false, cycle = false;

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
            SIZE_WINDOW = 720,
            LENGTH_ROBOT = 17.3984665354,
            WIDTH_ROBOT = 16.4220472441,
            SIZE_HALF_FIELD = 70.5,
            SIZE_TILE = 23.625,
            X_START_LEFT = SIZE_TILE * -1.5,
            X_START_RIGHT = SIZE_TILE * 0.5,
            Y_START = -SIZE_HALF_FIELD + LENGTH_ROBOT * 0.5,
            X_SHIFT_CENTER_AUDIENCE_STACK_CLEARANCE = -14,
            X_INTAKING = -52,
            Y_INTAKING_1 = -11.8125,
            Y_INTAKING_2 = -23.625,
            Y_INTAKING_3 = -35.4375,
            X_SHIFT_PRE_STACK_AUDIENCE_INNER_SPIKE = 6,
            Y_SHIFT_POST_INNER_SPIKE_BACKDROP = 1,
            TIME_SPIKE_BACKDROP = 0.75,
            TIME_PRE_SPIKE_AUDIENCE_PAINTBRUSH = 0.5,
            TIME_SPIKE_AUDIENCE = 1,
            TIME_SPIKE_TO_INTAKE_FLIP = 0.5,
            TIME_PRE_YELLOW = 0.5,
            X_SHIFT_INTAKING = 2,
            SPEED_INTAKING = 1,
            SPEED_INTAKE_STACK_APPROACH = 0.1,
            BOTTOM_ROW_HEIGHT = 2,
            X_BACKDROP = 50.1,
            Y_BACKDROP_0_BLUE = 43,
            Y_BACKDROP_0_RED = -28.5,
            WIDTH_PIXEL = 3.15,
            ANGLE_AWAY_TRUSS_SPIKE_APPROACH_RED = 5,
            ANGLE_AWAY_TRUSS_SPIKE_APPROACH_BLUE = 7.5,
            ANGLE_INNER_SPIKE_AUDIENCE_APPROACH = 1.3;

    public static EditablePose
            startPose = new EditablePose(X_START_RIGHT, Y_START, FORWARD),
            centerSpikeBackdrop = new EditablePose(15, -25, LEFT),
            innerSpikeBackdrop = new EditablePose(5.4, -35, LEFT),
            outerSpikeBackdrop = new EditablePose(28, -32, LEFT),
            centerSpikeAudience = new EditablePose(-47, -12, 3 * PI / 4.0),
            innerSpikeAudience = new EditablePose(-38, -30, LEFT),
            outerSpikeAudience = new EditablePose(-47, -12, FORWARD),
            postOuterAudience = new EditablePose(-36, -SIZE_TILE * .5, LEFT),
            parking = new EditablePose(X_BACKDROP, -60, LEFT),
            parked = new EditablePose(60, parking.y, LEFT),
            enteringBackstage = new EditablePose(22, -12, LEFT);

    private static Pose2d stackPos(int stack) {
        return new EditablePose(X_INTAKING, stack == 3 ? Y_INTAKING_3 : stack == 2 ? Y_INTAKING_2 : Y_INTAKING_1, LEFT).byAlliance().toPose2d();
    }

    private static void driveToStack1(TrajectorySequenceBuilder sequence, Intake.Height height) {
        sequence
                .addTemporalMarker(() -> {
                    robot.intake.setHeight(height);
                })
                .setTangent(LEFT)
                .splineTo(MainAuton.enteringBackstage.byAlliance().toPose2d().vec(), LEFT)
                .splineTo(stackPos(1).vec(), LEFT)
        ;
    }

    private static void driveToStack2(TrajectorySequenceBuilder sequence, Intake.Height height) {
        Pose2d turnToStack1 = new EditablePose(X_START_LEFT + X_SHIFT_CENTER_AUDIENCE_STACK_CLEARANCE, Y_INTAKING_1, LEFT).byAlliance().toPose2d();
        sequence
                .addTemporalMarker(() -> {
                    robot.intake.setHeight(height);
                })
                .setTangent(LEFT)
                .splineToConstantHeading(MainAuton.enteringBackstage.byAlliance().toPose2d().vec(), LEFT)
                .splineTo(turnToStack1.vec(), LEFT)
                .lineTo(postOuterAudience.byAlliance().toPose2d().vec())
                .lineTo(stackPos(2).vec())
        ;
    }

    private static void intake2Pixels(TrajectorySequenceBuilder sequence, Intake.Height height) {
        sequence
                .addTemporalMarker(() -> {
                    robot.intake.setMotorPower(SPEED_INTAKING);
                    while (robot.intake.colors[0] == Pixel.Color.EMPTY) {Thread.yield();}
                    robot.intake.setMotorPower(0);
                })
                .back(X_SHIFT_INTAKING)
                .addTemporalMarker(() -> {
                    robot.intake.setHeight(height.minus(1));
                })
                .forward(X_SHIFT_INTAKING)
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
                .lineToSplineHeading(MainAuton.enteringBackstage.byAlliance().toPose2d())

                .addTemporalMarker(() -> {
                    robot.deposit.lift.setTargetRow(first.y);
                })
                .splineTo(toPose2d(first).vec(), RIGHT)
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
            mTelemetry.addLine();
            mTelemetry.addLine();
            mTelemetry.addLine("Press both shoulder buttons to CONFIRM!");

            mTelemetry.update();
        }
        robot.preload();
        robot.initRun();

        TrajectorySequence[] sequences = generateTrajectories(partnerWillDoRand, cycle, ourPlacements);

        TeamPropDetector detector = new TeamPropDetector(hardwareMap);

        while (opModeInInit()) {
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
    private static TrajectorySequence[] generateTrajectories(boolean partnerWillDoRand, boolean cycle, int[] ourPlacements) {

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

            if (backdropSide) backdropPurpleYellow(sequence, placements, a, outer, inner);
            else audiencePurple(sequence, placements, a, outer, inner);

            if (cycle) {

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

    private static void backdropPurpleYellow(TrajectorySequenceBuilder sequence, ArrayList<Pixel> placements, double a, boolean outer, boolean inner) {
        if (inner) {
            Pose2d spike = innerSpikeBackdrop.byAlliance().toPose2d();
            sequence
                    .splineTo(spike.vec(), spike.getHeading())
                    .strafeRight(Y_SHIFT_POST_INNER_SPIKE_BACKDROP * a)
            ;
        } else {
            sequence.lineToSplineHeading((
                    outer ?
                            outerSpikeBackdrop.byAlliance() :
                            centerSpikeBackdrop.byAlliance()
            ).toPose2d());
        }

        sequence
                .addTemporalMarker(() -> {
                    robot.spike.toggle();
                })
                .waitSeconds(TIME_SPIKE_BACKDROP)
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
    }

    private static void audiencePurple(TrajectorySequenceBuilder sequence, ArrayList<Pixel> placements, double a, boolean outer, boolean inner) {
        sequence
                .UNSTABLE_addTemporalMarkerOffset(TIME_PRE_SPIKE_AUDIENCE_PAINTBRUSH, () -> {
                    robot.deposit.lift.setTargetRow(0);
                })
        ;

        Pose2d stack = stackPos(1);

        if (inner) {

            EditablePose preStack = new EditablePose(stack).clone();
            preStack.x += X_SHIFT_PRE_STACK_AUDIENCE_INNER_SPIKE;
            Pose2d spike = innerSpikeAudience.byAlliance().toPose2d();
            sequence
                    .setTangent(a * (REVERSE - ANGLE_INNER_SPIKE_AUDIENCE_APPROACH))
                    .splineToSplineHeading(spike, a * ANGLE_INNER_SPIKE_AUDIENCE_APPROACH)
                    .addTemporalMarker(() -> {
                        robot.deposit.paintbrush.dropPixel();
                    })
                    .waitSeconds(TIME_SPIKE_AUDIENCE)
                    .addTemporalMarker(() -> {
                        robot.deposit.lift.setTargetRow(-1);
                        robot.intake.toggle();
                    })
                    .lineToSplineHeading(preStack.toPose2d())
            ;

        } else if (outer) {

            Pose2d spike = outerSpikeAudience.byAlliance().toPose2d();
            sequence
                    .addTemporalMarker(() -> {
                        robot.intake.toggle();
                    })
                    .splineTo(spike.vec(), spike.getHeading())
                    .addTemporalMarker(() -> {
                        robot.deposit.paintbrush.dropPixel();
                    })
                    .waitSeconds(TIME_SPIKE_AUDIENCE)
                    .addTemporalMarker(() -> {
                        robot.deposit.lift.setTargetRow(-1);
                    })
                    .setTangent(a * FORWARD)
                    .turn(a * FORWARD)
            ;

        } else {

            Pose2d spike = centerSpikeAudience.byAlliance().toPose2d();
            sequence
                    .forward(10)
                    .addTemporalMarker(() -> {
                        robot.intake.toggle();
                    })
                    .splineTo(spike.vec(), spike.getHeading())
                    .addTemporalMarker(() -> {
                        robot.deposit.paintbrush.dropPixel();
                    })
                    .waitSeconds(TIME_SPIKE_AUDIENCE)
                    .addTemporalMarker(() -> {
                        robot.deposit.lift.setTargetRow(-1);
                    })
                    .setTangent(a * FORWARD)
                    .turn(a * PI / 4.0)
            ;

        }

        sequence
                .addTemporalMarker(() -> {
                    robot.deposit.paintbrush.toggleFloor();
                    robot.intake.setMotorPower(SPEED_INTAKE_STACK_APPROACH);
                })

                .lineToSplineHeading(stack)
                .setTangent(LEFT)

                .addTemporalMarker(() -> {
                    robot.intake.setMotorPower(SPEED_INTAKING);
                    while (robot.intake.colors[0] == Pixel.Color.EMPTY) {Thread.yield();}
                    robot.intake.setMotorPower(0);
                })
        ;

        score(sequence, placements, 0);
    }

    public static class EditablePose {

        public double x, y, heading;

        public static boolean backdropSide = true;

        public EditablePose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        public EditablePose(Pose2d pose) {
            this(pose.getX(), pose.getY(), pose.getHeading());
        }

        public EditablePose clone() {
            return new EditablePose(x, y, heading);
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
                    backdropSide ? x : X_START_RIGHT + X_START_LEFT - x,
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
