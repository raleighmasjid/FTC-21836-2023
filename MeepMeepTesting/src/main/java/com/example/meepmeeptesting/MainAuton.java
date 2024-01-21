package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Deposit.Paintbrush.TIME_DROP_FIRST;
import static com.example.meepmeeptesting.Deposit.Paintbrush.TIME_DROP_SECOND;
import static com.example.meepmeeptesting.Intake.Height.FIVE_STACK;
import static com.example.meepmeeptesting.Intake.Height.FOUR_STACK;
import static com.example.meepmeeptesting.MainAuton.EditablePose.backdropSide;
import static com.example.meepmeeptesting.Pixel.Color.WHITE;
import static com.example.meepmeeptesting.Pixel.Color.YELLOW;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;
import static java.util.Arrays.asList;
import static java.util.Collections.swap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.ArrayList;

public class MainAuton {

    static boolean isRed = false;
    static Backdrop autonBackdrop = new Backdrop();

    public static final double
            REVERSE = PI,
            LEFT = REVERSE,
            FORWARD = 1.5707963267948966,
            RIGHT = 0,
            BACKWARD = -1.5707963267948966;

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
            movingToStack2 = new EditablePose(X_START_LEFT + X_SHIFT_CENTER_AUDIENCE_STACK_CLEARANCE, -24, LEFT);

    private static Pose2d stackPos(int stack, Intake.Height height) {
        return new EditablePose(X_INTAKING + height.deltaX, stack == 3 ? Y_INTAKING_3 : stack == 2 ? movingToStack2.y : Y_INTAKING_1, LEFT).byAlliance().toPose2d();
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = MainAuton.startPose.byBoth().toPose2d();

        EditablePose rightSpike = new EditablePose(X_TILE - leftSpike.x, leftSpike.y, REVERSE - leftSpike.heading);

        ArrayList<Pixel> placements = new ArrayList<>(asList(
                new Pixel(3, 0, YELLOW),
                new Pixel(1, 0, WHITE),
                new Pixel(2, 0, WHITE),
                new Pixel(1, 1, WHITE),
                new Pixel(0, 1, WHITE),
                new Pixel(1, 2, WHITE),
                new Pixel(2, 1, WHITE),
                new Pixel(2, 2, WHITE),
                new Pixel(1, 3, WHITE),
                new Pixel(0, 3, WHITE),
                new Pixel(1, 4, WHITE),
                new Pixel(0, 5, WHITE),
                new Pixel(6, 0, WHITE),
                new Pixel(5, 0, WHITE),
                new Pixel(6, 1, WHITE),
                new Pixel(5, 1, WHITE),
                new Pixel(6, 2, WHITE),
                new Pixel(6, 3, WHITE)
        ));
        boolean partnerWillDoRand = false;
        PropDetectPipeline.Randomization rand = PropDetectPipeline.Randomization.CENTER;
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

        Pose2d turnToStack1 = new EditablePose(X_START_LEFT + X_SHIFT_CENTER_AUDIENCE_STACK_CLEARANCE, Y_INTAKING_1, LEFT).byAlliance().toPose2d();

        Pose2d afterSpike = new Pose2d(spike.getX() + X_SHIFT_BACKDROP_AFTER_SPIKE, spike.getY(), LEFT);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, toRadians(250), toRadians(250), 13.95)
//                .setConstraints(70, 90, toRadians(295), toRadians(295), 13.95)
                .setDimensions(16.42205, 17.39847)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startPose)
                                        .setTangent(startPose.getHeading())
                                        .splineTo(preSpike.vec(), preSpike.getHeading())
                                        .splineTo(spike.vec(), spike.getHeading())

                                        .setTangent(spike.getHeading() + REVERSE)
                                        .splineTo(postSpike.vec(), postSpike.getHeading() + REVERSE)
                                        .setTangent(FORWARD)
                                        .strafeRight((isRed ? 1 : -1) * X_SHIFT_CENTER_AUDIENCE_AFTER_SPIKE)
                                        .lineToSplineHeading(turnToStack1)
                                        .setTangent(LEFT)

                                        // INTAKING
                                        .addTemporalMarker(() -> {
//                                    robot.intake.setHeight(FIVE_STACK);
//                                    robot.intake.setRequiredIntakingAmount(1);
                                        })

                                        .splineTo(stackPos(1, FIVE_STACK).vec(), LEFT)
                                        .addTemporalMarker(() -> {
//                                    robot.intake.setMotorPower(1);
//                                    while (robot.intake.colors[0] == Pixel.Color.EMPTY) {Thread.yield();}
//                                    robot.intake.setMotorPower(0);
                                        })

                                        .lineTo(MainAuton.enteringBackstage.byAlliance().toPose2d().vec())

                                        .addTemporalMarker(() -> {
//                                    robot.deposit.lift.setTargetRow(placements.get(0).y);
                                        })
                                        .splineToConstantHeading(placements.get(0).toPose2d().vec(), MainAuton.startPose.byAlliance().heading + REVERSE)
                                        .addTemporalMarker(() -> {
//                                    robot.deposit.paintbrush.dropPixels(1);
                                            autonBackdrop.add(placements.get(0));
                                        })
                                        .waitSeconds(TIME_DROP_FIRST)

                                        .addTemporalMarker(() -> {
//                                    robot.deposit.lift.setTargetRow(placements.get(0 + 1).y);
                                        })
                                        .lineToConstantHeading(placements.get(0 + 1).toPose2d().vec())
                                        .addTemporalMarker(() -> {
//                                    robot.deposit.paintbrush.dropPixels(2);
                                            autonBackdrop.add(placements.get(0 + 1));
                                        })
                                        .waitSeconds(TIME_DROP_SECOND)

                                        .addTemporalMarker(() -> {
//                                    robot.intake.setHeight(FOUR_STACK);
                                        })
                                        .setTangent(MainAuton.startPose.byAlliance().heading)
                                        .splineToConstantHeading(MainAuton.enteringBackstage.byAlliance().toPose2d().vec(), LEFT)
                                        .splineTo(stackPos(1, FOUR_STACK).vec(), LEFT)


                                        .addTemporalMarker(() -> {
//                                    robot.intake.setMotorPower(1);
//                                    while (robot.intake.colors[0] == Pixel.Color.EMPTY) {Thread.yield();}
//                                    robot.intake.setMotorPower(0);
                                        })
                                        .back(X_SHIFT_INTAKING)
                                        .addTemporalMarker(() -> {
//                                    robot.intake.setHeight(FOUR_STACK.minus(1));
                                        })
                                        .lineTo(stackPos(1, FOUR_STACK.minus(1)).vec())
                                        .addTemporalMarker(() -> {
//                                    robot.intake.setMotorPower(1);
//                                    while (robot.intake.colors[1] == Pixel.Color.EMPTY) {Thread.yield();}
//                                    robot.intake.setMotorPower(0);
                                        })


                                        .lineTo(MainAuton.enteringBackstage.byAlliance().toPose2d().vec())

                                        .addTemporalMarker(() -> {
//                                    robot.deposit.lift.setTargetRow(placements.get(2).y);
                                        })
                                        .splineToConstantHeading(placements.get(2).toPose2d().vec(), MainAuton.startPose.byAlliance().heading + REVERSE)
                                        .addTemporalMarker(() -> {
//                                    robot.deposit.paintbrush.dropPixels(1);
                                            autonBackdrop.add(placements.get(2));
                                        })
                                        .waitSeconds(TIME_DROP_FIRST)

                                        .addTemporalMarker(() -> {
//                                    robot.deposit.lift.setTargetRow(placements.get(2 + 1).y);
                                        })
                                        .lineToConstantHeading(placements.get(2 + 1).toPose2d().vec())
                                        .addTemporalMarker(() -> {
//                                    robot.deposit.paintbrush.dropPixels(2);
                                            autonBackdrop.add(placements.get(2 + 1)).print();
                                        })
                                        .waitSeconds(TIME_DROP_SECOND)

                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(.85f)
                .addEntity(myBot)
                .start();
    }

    public static class EditablePose {

        public double x, y, heading;

        static boolean backdropSide = false;

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