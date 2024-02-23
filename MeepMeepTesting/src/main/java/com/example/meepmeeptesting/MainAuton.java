package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.AutonCycles.driveToStack1;
import static com.example.meepmeeptesting.AutonPreloads.audiencePreloadsAndWhite;
import static com.example.meepmeeptesting.AutonPreloads.backdropPreloads;
import static com.example.meepmeeptesting.AutonVars.LENGTH_ROBOT;
import static com.example.meepmeeptesting.AutonVars.SIZE_WINDOW;
import static com.example.meepmeeptesting.AutonVars.WIDTH_PIXEL;
import static com.example.meepmeeptesting.AutonVars.WIDTH_ROBOT;
import static com.example.meepmeeptesting.AutonVars.X_BACKDROP;
import static com.example.meepmeeptesting.AutonVars.Y_BACKDROP_0_BLUE;
import static com.example.meepmeeptesting.AutonVars.Y_BACKDROP_0_RED;
import static com.example.meepmeeptesting.AutonVars.backdropSide;
import static com.example.meepmeeptesting.AutonVars.parked;
import static com.example.meepmeeptesting.AutonVars.parking;
import static com.example.meepmeeptesting.Intake.Height.FIVE_STACK;
import static com.example.meepmeeptesting.Intake.Height.FOUR_STACK;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;
import static java.util.Collections.swap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.example.meepmeeptesting.placementalg.AutonPixelSupplier;
import com.example.meepmeeptesting.placementalg.Backdrop;
import com.example.meepmeeptesting.placementalg.Pixel;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;

public class MainAuton {

    static boolean isRed = true;
    static Backdrop autonBackdrop = new Backdrop();

    public static final double
            REVERSE = PI,
            LEFT = REVERSE,
            FORWARD = 1.5707963267948966,
            RIGHT = 0,
            BACKWARD = -1.5707963267948966;

    private static final int[] ourPlacements = {1, 3, 6};
    private static boolean partnerWillDoRand = false, cycle = false;

    /**
     * @return A {@link Pose2d} corresponding to the phsyical scoring location of this {@link Pixel}
     */
    public static Pose2d toPose2d(Pixel pixel) {
        return new Pose2d(
                X_BACKDROP,
                (isRed ? Y_BACKDROP_0_RED : Y_BACKDROP_0_BLUE)
                        - ((pixel.x - 1) * WIDTH_PIXEL)
                        - (pixel.y % 2 != 0 ? 0.5 * WIDTH_PIXEL : 0
                ),
                PI
        );
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep((int) SIZE_WINDOW);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, toRadians(250), toRadians(250), 13.95)
//                .setConstraints(70, 90, toRadians(295), toRadians(295), 13.95)
                .setDimensions(WIDTH_ROBOT, LENGTH_ROBOT)
                .followTrajectorySequence(robotdrivetrain -> {

                    Pose2d startPose = AutonVars.startPose.byBoth().toPose2d();
                    // robot.drivetrain.setPoseEstimate(startPose);

                    PropDetectPipeline.Randomization rand = PropDetectPipeline.Randomization.RIGHT;


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

                    TrajectorySequenceBuilder sequence = robotdrivetrain.trajectorySequenceBuilder(startPose)
                            .setTangent(startPose.getHeading())
                            ;

                    if (backdropSide) backdropPreloads(sequence, placements, a, outer, inner);
                    else audiencePreloadsAndWhite(sequence, placements, a, outer, inner);

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
                    
                    return sequence.build();
                });

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(.85f)
                .addEntity(myBot)
                .start();
    }
}