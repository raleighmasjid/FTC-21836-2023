package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        boolean isRed = false;
        double side = isRed ? 1 : -1;
        double startAngle = toRadians(30 * -side + 180);

        Pixel pixel1 = new Pixel(1, 0);
        Pixel pixel2 = new Pixel(5, 0);
        Pose2d scoringPose1 = pixel1.toPose2d(isRed);
        Pose2d scoringPose2 = pixel2.toPose2d(isRed);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, toRadians(180), toRadians(180), 15)
                .setDimensions(16.51929, 17.39847)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, startAngle))
                                .setReversed(true)
                                .splineTo(scoringPose1.vec(), scoringPose1.getHeading())
                                .waitSeconds(1)
                                .lineTo(scoringPose2.vec())
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}