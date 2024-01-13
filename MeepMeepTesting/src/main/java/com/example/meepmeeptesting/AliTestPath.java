package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AliTestPath {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(26, 62, Math.toRadians(-90)))
//                                .lineToSplineHeading(new Pose2d(-55, 24, Math.toRadians(-180)))
                                .splineTo(new Vector2d(0, 35), Math.toRadians(-180))
                                .forward(57)
                                .back(107)
                                .strafeRight(6)
                                .waitSeconds(1) // Waits 1 seconds
                                .strafeLeft(6)
                                .forward(107)
                                .back(107)
                                .strafeRight(6)
                                .waitSeconds(1) // Waits 1 seconds
                                .strafeLeft(6)
                                .forward(107)
                                .back(107)
                                .strafeRight(6)
                                .waitSeconds(1) // Waits 1 seconds
                                .strafeLeft(6)
                                .forward(107)
                                .back(107)
                                .strafeRight(6)
                                .waitSeconds(1) // Waits 1 seconds
                                .strafeLeft(6)
                                .waitSeconds(10) // Waits 10 seconds

                                .build()

                );

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

