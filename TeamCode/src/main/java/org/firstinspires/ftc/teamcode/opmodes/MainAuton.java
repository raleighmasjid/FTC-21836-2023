package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.randomizations;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.EditablePose.backdropSide;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_FIRST;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_SECOND;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FIVE_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FOUR_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRed;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;
import static java.util.Collections.swap;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.AutonPixelSupplier;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel;
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
    static Pose2d autonEndPose = new Pose2d(0, 0, FORWARD);

    public static boolean keyPressed(int gamepad, GamepadKeys.Button button) {
        return (gamepad == 2 ? gamepadEx2 : gamepadEx1).wasJustPressed(button);
    }

    public static double
            X_START_LEFT = -35,
            X_START_RIGHT = 12,
            X_SHIFT_BACKDROP_AFTER_SPIKE = 12,
            Y_SHIFT_BEFORE_SPIKE = 17,
            Y_SHIFT_AFTER_SPIKE = 26,
            Y_SHIFT_AUDIENCE_AFTER_SPIKE = 16,
            X_SHIFT_CENTER_AUDIENCE_AFTER_SPIKE = -22,
            X_SHIFT_CENTER_AUDIENCE_STACK_CLEARANCE = -14,
            X_TILE = 24,
            X_INTAKING = -56,
            Y_INTAKING_1 = -12,
            Y_INTAKING_3 = -36,
            CYCLES_BACKDROP_SIDE = 0,
            CYCLES_AUDIENCE_SIDE = 0,
            TIME_SPIKE_TO_INTAKE_FLIP = 0.5,
            X_SHIFT_INTAKING = 5;

    public static EditablePose
            startPose = new EditablePose(X_START_RIGHT, -61.788975, FORWARD),
            centerSpike = new EditablePose(X_START_RIGHT, -26, startPose.heading),
            leftSpike = new EditablePose(2.5, -36, toRadians(150)),
            parking = new EditablePose(Backdrop.X, -60, LEFT),
            parked = new EditablePose(60, parking.y, LEFT),
            enteringBackstage = new EditablePose(12, -12, LEFT),
            movingToStack2 = new EditablePose(-45, -24, LEFT);

    private static Pose2d stackPos(int stack, Intake.Height height) {
        return new EditablePose(X_INTAKING + height.deltaX, stack == 3 ? Y_INTAKING_3 : stack == 2 ? movingToStack2.y : Y_INTAKING_1, LEFT).byAlliance().toPose2d();
    }

    private static void driveToStack1(TrajectorySequenceBuilder sequence, Intake.Height height) {
        sequence
                .addTemporalMarker(() -> {
                    robot.intake.setHeight(height);
                })
                .setTangent(MainAuton.startPose.byAlliance().heading)
                .splineToConstantHeading(MainAuton.enteringBackstage.byAlliance().toPose2d().vec(), LEFT)
                .splineTo(stackPos(1, height).vec(), LEFT)
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
                .lineTo(stackPos(2, height).vec())
        ;
    }

    private static void intake2Pixels(TrajectorySequenceBuilder sequence, int stack, Intake.Height height) {
        Intake.Height height2 = height.minus(1);
        sequence
                .addTemporalMarker(() -> {
                        robot.intake.setMotorPower(1);
                        while (robot.intake.colors[0] == Pixel.Color.EMPTY) {Thread.yield();}
                        robot.intake.setMotorPower(0);
                })
                .back(X_SHIFT_INTAKING)
                .addTemporalMarker(() -> {
                        robot.intake.setHeight(height2);
                })
                .lineTo(stackPos(stack, height2).vec())
                .addTemporalMarker(() -> {
                        robot.intake.setMotorPower(1);
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
                .splineToConstantHeading(first.toPose2d().vec(), MainAuton.startPose.byAlliance().heading + REVERSE)
                .addTemporalMarker(() -> {
                    robot.deposit.paintbrush.dropPixels(1);
                    autonBackdrop.add(first);
                })
                .waitSeconds(TIME_DROP_FIRST)

                .addTemporalMarker(() -> {
                    robot.deposit.lift.setTargetRow(second.y);
                })
                .lineToConstantHeading(second.toPose2d().vec())
                .addTemporalMarker(() -> {
                    robot.deposit.paintbrush.dropPixels(2);
                    autonBackdrop.add(second);
                })
                .waitSeconds(TIME_DROP_SECOND)
        ;
    }

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

        boolean partnerWillDoRand = false, doCycles = true;
        // Get gamepad 1 button input and save alliance and side for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (keyPressed(1, B))           isRed = true;
            if (keyPressed(1, X))           isRed = false;
            if (keyPressed(1, DPAD_RIGHT))  backdropSide = isRed;
            if (keyPressed(1, DPAD_LEFT))   backdropSide = !isRed;
            if (keyPressed(1, DPAD_UP))     doCycles = true;
            if (keyPressed(1, DPAD_DOWN))   doCycles = false;
            if (keyPressed(1, Y))           partnerWillDoRand = !partnerWillDoRand;
            mTelemetry.addLine("Selected " + (isRed ? "RED " : "BLUE ") + (backdropSide ? "BACKDROP " : "AUDIENCE ") + "side");
            mTelemetry.addLine();
            mTelemetry.addLine("Your alliance PARTNER WILL " + (partnerWillDoRand ? "" : "NOT ") + "PLACE YELLOW");
            mTelemetry.addLine();
            mTelemetry.addLine("You WILL " + (doCycles ? "CYCLE" : "PARK"));
            mTelemetry.addLine();
            mTelemetry.addLine("Press both shoulder buttons to CONFIRM!");
            mTelemetry.update();
        }
        mTelemetry.addLine("Confirmed " + (isRed ? "RED" : "BLUE") + " " + (backdropSide ? "BACKDROP" : "AUDIENCE") + " side");
        mTelemetry.update();

        Pose2d startPose = MainAuton.startPose.byBoth().toPose2d();
        robot.drivetrain.setPoseEstimate(startPose);

        PropDetectPipeline.Randomization location = PropDetectPipeline.Randomization.RIGHT;
        TeamPropDetector detector = new TeamPropDetector(hardwareMap);

        TrajectorySequence[] sequences = new TrajectorySequence[3];
        EditablePose rightSpike = new EditablePose(X_TILE - leftSpike.x, leftSpike.y, LEFT - leftSpike.heading);

        for (PropDetectPipeline.Randomization rand : randomizations) {

            ArrayList<Pixel> placements = AutonPixelSupplier.getPlacements(rand, partnerWillDoRand);
            if (partnerWillDoRand) placements.remove(0);
            if (!backdropSide) swap(placements, 0, 1);

            Pose2d spike;
            switch (rand) {
                case LEFT:
                    spike = (isRed ? leftSpike : rightSpike).byBoth().toPose2d();
                    break;
                case RIGHT:
                    spike = (isRed ? rightSpike : leftSpike).byBoth().toPose2d();
                    break;
                case CENTER:
                default:
                    spike = centerSpike.byBoth().toPose2d();
            }

            Pose2d preSpike = new EditablePose(
                    MainAuton.startPose.x,
                    MainAuton.startPose.y + Y_SHIFT_BEFORE_SPIKE,
                    MainAuton.startPose.heading
            ).byBoth().toPose2d();

            Pose2d postSpike = new EditablePose(
                    MainAuton.startPose.x,
                    MainAuton.startPose.y + Y_SHIFT_AFTER_SPIKE,
                    MainAuton.startPose.heading
            ).byBoth().toPose2d();

            Pose2d turnToStack1 = new EditablePose(MainAuton.startPose.x + X_SHIFT_CENTER_AUDIENCE_STACK_CLEARANCE, Y_INTAKING_1, LEFT).byBoth().toPose2d();

            TrajectorySequenceBuilder sequence = robot.drivetrain.trajectorySequenceBuilder(startPose)
                    .setTangent(startPose.getHeading())
                    .splineTo(preSpike.vec(), preSpike.getHeading())
                    .splineTo(spike.vec(), spike.getHeading())
            ;

            if (backdropSide) {

                Pose2d afterSpike = new Pose2d(spike.getX() + X_SHIFT_BACKDROP_AFTER_SPIKE, spike.getY(), LEFT);

                sequence
                        .setTangent(afterSpike.getHeading())
                        .lineToSplineHeading(afterSpike)
                        .UNSTABLE_addTemporalMarkerOffset(TIME_SPIKE_TO_INTAKE_FLIP, () -> {
                            robot.intake.setRequiredIntakingAmount(2);
                            robot.deposit.lift.setTargetRow(placements.get(0).y);
                        })
                        .splineToSplineHeading(placements.get(0).toPose2d(), RIGHT)
                        .addTemporalMarker(() -> {
                            robot.deposit.paintbrush.dropPixels(1);
                            autonBackdrop.add(placements.get(0));
                        })
                        .waitSeconds(TIME_DROP_SECOND)
                ;

            } else {

                sequence.setTangent(spike.getHeading() + REVERSE);

                switch (rand) {
                    case LEFT:
                    case RIGHT:
                        sequence
                                .splineTo(preSpike.vec(), preSpike.getHeading() + REVERSE)
                                .setTangent(FORWARD)
                                .forward(Y_SHIFT_AUDIENCE_AFTER_SPIKE)
                        ;
                        break;
                    case CENTER:
                        sequence
                                .splineTo(postSpike.vec(), postSpike.getHeading() + REVERSE)
                                .setTangent(FORWARD)
                                .strafeRight((isRed ? 1 : -1) * X_SHIFT_CENTER_AUDIENCE_AFTER_SPIKE)
                                .lineToSplineHeading(turnToStack1)
                                .setTangent(LEFT)
                        ;
                }

                sequence
                        // INTAKING
                        .addTemporalMarker(() -> {
                            robot.intake.setHeight(FIVE_STACK);
                            robot.intake.setRequiredIntakingAmount(1);
                        })

                        .splineTo(stackPos(1, FIVE_STACK).vec(), LEFT)
                        .addTemporalMarker(() -> {
                            robot.intake.setMotorPower(1);
                            while (robot.intake.colors[0] == Pixel.Color.EMPTY) {Thread.yield();}
                            robot.intake.setMotorPower(0);
                        })
                ;
                score(sequence, placements, 0);

            }

            if (!doCycles) {
                sequence
                        .lineTo(parking.byAlliance().toPose2d().vec())
                        .lineTo(parked.byAlliance().toPose2d().vec())
                ;
            } else {

                Intake.Height height = backdropSide ? FIVE_STACK : FOUR_STACK;
                int placement = backdropSide ? 1 : 2;

                // CYCLE 1
                driveToStack1(sequence, height);
                intake2Pixels(sequence, 1, height);
                score(sequence, placements, placement);

                // CYCLE 2
                if (backdropSide) {
                    driveToStack1(sequence, height.minus(2));
                    intake2Pixels(sequence, 1, height.minus(2));
                    score(sequence, placements, placement + 2);
                }

                // CYCLE 3
//                driveToStack2(sequence, FIVE_STACK);
//                intake2From1Stack(sequence, 2, FIVE_STACK);
//                score(sequence, placements, placement + 4);
            }

            sequences[rand.ordinal()] = sequence.build();
        }

        while (opModeInInit()) {
            mTelemetry.addData("Location", (location = detector.run()).name());
            mTelemetry.update();
        }

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
