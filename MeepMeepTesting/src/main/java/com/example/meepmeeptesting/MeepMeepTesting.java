package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Pixel.Color.WHITE;
import static com.example.meepmeeptesting.Pixel.Color.YELLOW;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public enum Randomization {
        LEFT,
        CENTER,
        RIGHT
    }

    static Randomization rand = Randomization.LEFT;
    static boolean isRed = true, isRight = true;
    static Backdrop backdrop = new Backdrop();

    public static double
            X_START_RIGHT = 12,
            X_START_LEFT = -35,
            Y_START = -61.788975,
            Y_CENTER_SPIKE = -28;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    static EditablePose
            startPose = new EditablePose(X_START_RIGHT, Y_START, FORWARD),
            centerSpike = new EditablePose(X_START_RIGHT, Y_CENTER_SPIKE, FORWARD);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pixel[] placements = {
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
                new Pixel(6, 9, WHITE),
        };

        double alliance = isRed ? 1 : -1;
        Pose2d startPose = MeepMeepTesting.startPose.byBoth().toPose2d();
        Pose2d centerSpike = MeepMeepTesting.centerSpike.byBoth().toPose2d();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, toRadians(180), toRadians(180), 13.2686055118)
                .setDimensions(16.42205, 17.39847)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .forward(24)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(.85f)
                .addEntity(myBot)
                .start();
    }

    private static class EditablePose {

        public double x, y, heading;

        private EditablePose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        private EditablePose byAlliance() {
            double alliance = isRed ? 1 : -1;
            y *= alliance;
            heading *= alliance;
            return this;
        }

        private EditablePose bySide() {
            boolean isRight = MeepMeepTesting.isRight == isRed;
            x += isRight ? 0 : X_START_LEFT - X_START_RIGHT;
            return this;
        }

        private EditablePose byBoth() {
            return byAlliance().bySide();
        }

        private Pose2d toPose2d() {
            return new Pose2d(x, y, heading);
        }
    }
}