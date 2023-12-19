package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.Deposit.Paintbrush.TIME_DROP_FIRST;
import static com.example.meepmeeptesting.Deposit.Paintbrush.TIME_DROP_SECOND;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static int
            X1 = 1,
            Y1 = 0,
            X2 = 5,
            Y2 = 0;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        boolean isRed = true;

        Pixel[] placements = new Pixel[]{
                new Pixel(X1, Y1),
                new Pixel(X2, Y2)
        };

        Pose2d scoringPos1 = placements[0].toPose2d(isRed);
        Pose2d scoringPos2 = placements[1].toPose2d(isRed);

        Pose2d startPose = new Pose2d(0, 0, toRadians(160));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, toRadians(180), toRadians(180), 15)
                .setDimensions(16.51929, 17.39847)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .addTemporalMarker(() -> {
//                                    robot.deposit.lift.setTargetRow(placements[0].y);
                                })
                                .lineToSplineHeading(scoringPos1)
                                .addTemporalMarker(() -> {
//                                    robot.deposit.paintbrush.dropPixels(1);
//                                    latestScan.add(placements[0]);
                                })
                                .waitSeconds(TIME_DROP_FIRST)
                                .addTemporalMarker(() -> {
//                                    robot.deposit.lift.setTargetRow(placements[1].y);
                                })
                                .lineToConstantHeading(scoringPos2.vec())
                                .addTemporalMarker(() -> {
//                                    robot.deposit.paintbrush.dropPixels(2);
//                                    latestScan.add(placements[1]);
                                })
                                .waitSeconds(TIME_DROP_SECOND)
                                .addTemporalMarker(() -> {
//                                    trajectoryReady = false;
                                })
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}