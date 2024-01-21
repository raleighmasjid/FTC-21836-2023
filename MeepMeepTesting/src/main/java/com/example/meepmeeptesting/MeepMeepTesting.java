package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Deposit.Paintbrush.TIME_DROP_FIRST;
import static com.example.meepmeeptesting.Deposit.Paintbrush.TIME_DROP_SECOND;
import static com.example.meepmeeptesting.MeepMeepTesting.EditablePose.backdropSide;
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

public class MeepMeepTesting {

    static boolean isRed = true;
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
            X_SHIFT_AFTER_SPIKE = 5,
            Y_SHIFT_BEFORE_SPIKE = 17,
            X_SHIFT_AUDIENCE_AFTER_SPIKE = 22,
            Y_SHIFT_AUDIENCE_AFTER_SPIKE = 33,
            X_TILE = 24,
            X_INTAKING = -56,
            Y_INTAKING_1 = -12,
            Y_INTAKING_2 = -23,
            Y_INTAKING_3 = -35,
            CYCLES_BACKDROP_SIDE = 0,
            CYCLES_AUDIENCE_SIDE = 0,
            TIME_SPIKE_TO_INTAKE_FLIP = 0.5;

    public static EditablePose
            startPose = new EditablePose(X_START_RIGHT, -61.788975, FORWARD),
            centerSpike = new EditablePose(X_START_RIGHT, -26, startPose.heading),
            leftSpike = new EditablePose(2.5, -36, toRadians(150)),
            parking = new EditablePose(Backdrop.X, -60, LEFT),
            parked = new EditablePose(60, parking.y, LEFT),
            turnToStackPos = new EditablePose(-48, startPose.y + Y_SHIFT_BEFORE_SPIKE + Y_SHIFT_AUDIENCE_AFTER_SPIKE, LEFT),
            enteringBackstage = new EditablePose(12, -12, LEFT);

    public static Pose2d stackPos(int stack) {
        return new EditablePose(X_INTAKING, stack == 3 ? Y_INTAKING_3 : stack == 2 ? Y_INTAKING_2 : Y_INTAKING_1, LEFT).byAlliance().toPose2d();
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = MeepMeepTesting.startPose.byBoth().toPose2d();

        EditablePose rightSpike = new EditablePose(X_TILE - leftSpike.x, leftSpike.y, REVERSE - leftSpike.heading);

        ArrayList<Pixel> placements = new ArrayList<>(asList(
                new Pixel(1, 0, YELLOW),
                new Pixel(6, 0, WHITE),
                new Pixel(5, 0, WHITE),
                new Pixel(6, 1, WHITE),
                new Pixel(5, 1, WHITE),
                new Pixel(6, 2, WHITE),
                new Pixel(4, 0, WHITE),
                new Pixel(4, 1, WHITE),
                new Pixel(5, 2, WHITE),
                new Pixel(6, 3, WHITE),
                new Pixel(5, 3, WHITE),
                new Pixel(6, 4, WHITE),
                new Pixel(6, 5, WHITE),
                new Pixel(3, 0, WHITE),
                new Pixel(2, 0, WHITE),
                new Pixel(3, 1, WHITE),
                new Pixel(2, 1, WHITE),
                new Pixel(4, 2, WHITE),
                new Pixel(3, 2, WHITE),
                new Pixel(4, 3, WHITE),
                new Pixel(3, 3, WHITE),
                new Pixel(5, 4, WHITE),
                new Pixel(4, 4, WHITE),
                new Pixel(5, 5, WHITE),
                new Pixel(4, 5, WHITE),
                new Pixel(6, 6, WHITE),
                new Pixel(5, 6, WHITE),
                new Pixel(6, 7, WHITE),
                new Pixel(5, 7, WHITE),
                new Pixel(6, 8, WHITE),
                new Pixel(6, 9, WHITE)
        ));
        boolean partnerWillDoRand = false;
        PropDetectPipeline.Randomization rand = PropDetectPipeline.Randomization.RIGHT;
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
                MeepMeepTesting.startPose.x,
                MeepMeepTesting.startPose.y + Y_SHIFT_BEFORE_SPIKE,
                MeepMeepTesting.startPose.heading
        ).byBoth().toPose2d();

        Pose2d turnToStackPos = MeepMeepTesting.turnToStackPos.byAlliance().toPose2d();
        Pose2d enteringBackstage = MeepMeepTesting.enteringBackstage.byAlliance().toPose2d();

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
                                .splineTo(preSpike.vec(), preSpike.getHeading() + REVERSE)
                                .strafeLeft((isRed ? 1 : -1) * X_SHIFT_AUDIENCE_AFTER_SPIKE)
                                .forward(Y_SHIFT_AUDIENCE_AFTER_SPIKE)
                                .lineToSplineHeading(turnToStackPos)
                                .addTemporalMarker(() -> {
//                                    robot.intake.setHeight(FIVE_STACK);
//                                    robot.intake.setRequiredIntakingAmount(1);
                                })
                                .lineTo(stackPos(1).vec())
                                .addTemporalMarker(() -> {
//
                                })
                                .lineTo(enteringBackstage.vec())

                                .addTemporalMarker(() -> {
//                                    robot.deposit.lift.setTargetRow(placements.get(0).y);
//                                    robot.intake.setRequiredIntakingAmount(2);
                                })
                                .splineToConstantHeading(placements.get(0).toPose2d().vec(), startPose.getHeading() + REVERSE)
                                .addTemporalMarker(() -> {
//                                    robot.deposit.paintbrush.dropPixels(1);
                                    autonBackdrop.add(placements.get(0));
                                })
                                .waitSeconds(TIME_DROP_FIRST)

                                .addTemporalMarker(() -> {
//                                    robot.deposit.lift.setTargetRow(placements.get(1).y);
                                })
                                .lineToConstantHeading(placements.get(1).toPose2d().vec())
                                .addTemporalMarker(() -> {
//                                    robot.deposit.paintbrush.dropPixels(2);
                                    autonBackdrop.add(placements.get(1));
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