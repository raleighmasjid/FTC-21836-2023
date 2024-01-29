package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Deposit.Paintbrush.TIME_DROP_SECOND;
import static com.example.meepmeeptesting.Pixel.Color.WHITE;
import static com.example.meepmeeptesting.Pixel.Color.YELLOW;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoScoreTesting {

    static Backdrop backdrop = new Backdrop();

    public static final double
            LEFT = PI,
            FORWARD = 1.5707963267948966,
            RIGHT = 0,
            BACKWARD = -1.5707963267948966;

    public static MainAuton.EditablePose
            startPose = new MainAuton.EditablePose(24, -16, LEFT);

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
                new Pixel(6, 9, WHITE),};

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, toRadians(250), toRadians(250), 13.95)
                .setDimensions(16.42205, 17.39847)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose.byAlliance().toPose2d())
                                .addTemporalMarker(() -> {
//                                    robot.deposit.lift.setTargetRow(placements[1].y);
                                })
                                .lineToSplineHeading(placements[0].toPose2d())
                                .addTemporalMarker(() -> {
//                                    robot.deposit.paintbrush.dropPixels(2);
//                                    latestScan.add(placements[1]);
//                                    justScored = true;
                                })
                                .waitSeconds(TIME_DROP_SECOND)
                                .addTemporalMarker(() -> {
//                                    trajectoryReady = false;
                                })
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(.85f)
                .addEntity(myBot)
                .start();
    }
}