package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.AutonCycles.score;
import static com.example.meepmeeptesting.AutonCycles.stackPos;
import static com.example.meepmeeptesting.AutonVars.ANGLE_INNER_SPIKE_AUDIENCE_APPROACH;
import static com.example.meepmeeptesting.AutonVars.ANGLE_OUTER_SPIKE_APPROACH_BLUE;
import static com.example.meepmeeptesting.AutonVars.ANGLE_OUTER_SPIKE_APPROACH_RED;
import static com.example.meepmeeptesting.AutonVars.TIME_INTAKING;
import static com.example.meepmeeptesting.AutonVars.TIME_PRE_SPIKE_AUDIENCE_PAINTBRUSH;
import static com.example.meepmeeptesting.AutonVars.TIME_PRE_YELLOW;
import static com.example.meepmeeptesting.AutonVars.TIME_SPIKE_AUDIENCE;
import static com.example.meepmeeptesting.AutonVars.TIME_SPIKE_BACKDROP;
import static com.example.meepmeeptesting.AutonVars.TIME_SPIKE_TO_INTAKE_FLIP;
import static com.example.meepmeeptesting.AutonVars.X_SHIFT_PRE_STACK_AUDIENCE_INNER_SPIKE;
import static com.example.meepmeeptesting.AutonVars.Y_SHIFT_POST_INNER_SPIKE_BACKDROP;
import static com.example.meepmeeptesting.AutonVars.centerSpikeAudience;
import static com.example.meepmeeptesting.AutonVars.centerSpikeBackdrop;
import static com.example.meepmeeptesting.AutonVars.innerSpikeAudience;
import static com.example.meepmeeptesting.AutonVars.innerSpikeBackdrop;
import static com.example.meepmeeptesting.AutonVars.isRed;
import static com.example.meepmeeptesting.AutonVars.offsetAudienceCenter;
import static com.example.meepmeeptesting.AutonVars.offsetAudienceInner;
import static com.example.meepmeeptesting.AutonVars.offsetAudienceOuter;
import static com.example.meepmeeptesting.AutonVars.outerSpikeAudience;
import static com.example.meepmeeptesting.AutonVars.outerSpikeBackdrop;
import static com.example.meepmeeptesting.Deposit.Paintbrush.TIME_DROP_SECOND;
import static com.example.meepmeeptesting.MainAuton.FORWARD;
import static com.example.meepmeeptesting.MainAuton.LEFT;
import static com.example.meepmeeptesting.MainAuton.REVERSE;
import static com.example.meepmeeptesting.MainAuton.RIGHT;
import static com.example.meepmeeptesting.MainAuton.autonBackdrop;
import static com.example.meepmeeptesting.MainAuton.toPose2d;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.example.meepmeeptesting.placementalg.Pixel;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;

class AutonPreloads {

    static void backdropPreloads(TrajectorySequenceBuilder sequence, ArrayList<Pixel> placements, double a, boolean outer, boolean inner) {
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
                    // robot.spike.toggle();
                })
                .waitSeconds(TIME_SPIKE_BACKDROP)
                .setTangent(RIGHT)
                .UNSTABLE_addTemporalMarkerOffset(TIME_SPIKE_TO_INTAKE_FLIP, () -> {
                    // robot.deposit.lift.setTargetRow(placements.get(0).y);
                })
                .splineToConstantHeading(toPose2d(placements.get(0)).vec(),
                        outer ?
                                isRed ? ANGLE_OUTER_SPIKE_APPROACH_RED : ANGLE_OUTER_SPIKE_APPROACH_BLUE :
                                RIGHT
                )
                .waitSeconds(TIME_PRE_YELLOW)
                .addTemporalMarker(() -> {
                    // robot.deposit.paintbrush.dropPixel();
                    autonBackdrop.add(placements.get(0));
                })
                .waitSeconds(TIME_DROP_SECOND)
        ;
    }

    static void audiencePreloadsAndWhite(TrajectorySequenceBuilder sequence, ArrayList<Pixel> placements, double a, boolean outer, boolean inner) {

        Pose2d offset = (
                inner ? offsetAudienceInner :
                        outer ? offsetAudienceOuter :
                                offsetAudienceCenter
        ).byAlliance().toPose2d();

        sequence
                .UNSTABLE_addTemporalMarkerOffset(TIME_PRE_SPIKE_AUDIENCE_PAINTBRUSH, () -> {
                    // robot.deposit.lift.setTargetRow(HEIGHT_SPIKE_AUDIENCE);
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
                    .strafeLeft(a * 2)
                    .addTemporalMarker(() -> {
                        // robot.deposit.paintbrush.dropPixel();
                    })
                    .waitSeconds(TIME_SPIKE_AUDIENCE)
                    .addTemporalMarker(() -> {
                        // robot.deposit.lift.setTargetRow(-1);
                        // robot.intake.toggle();
                    })
                    .lineToSplineHeading(preStack.toPose2d())
            ;

        } else if (outer) {

            Pose2d spike = outerSpikeAudience.byAlliance().toPose2d();
            sequence
                    .addTemporalMarker(() -> {
                        // robot.intake.toggle();
                    })
                    .splineTo(spike.vec(), spike.getHeading())
                    .addTemporalMarker(() -> {
                        // robot.deposit.paintbrush.dropPixel();
                    })
                    .waitSeconds(TIME_SPIKE_AUDIENCE)
                    .addTemporalMarker(() -> {
                        // robot.deposit.lift.setTargetRow(-1);
                    })
                    .setTangent(a * FORWARD)
                    .turn(a * FORWARD)
            ;

        } else {

            Pose2d spike = centerSpikeAudience.byAlliance().toPose2d();
            sequence
                    .forward(10)
                    .addTemporalMarker(() -> {
                        // robot.intake.toggle();
                    })
                    .splineTo(spike.vec(), spike.getHeading())
                    .addTemporalMarker(() -> {
                        // robot.deposit.paintbrush.dropPixel();
                    })
                    .waitSeconds(TIME_SPIKE_AUDIENCE)
                    .addTemporalMarker(() -> {
                        // robot.deposit.lift.setTargetRow(-1);
                    })
                    .setTangent(a * FORWARD)
                    .turn(a * PI / 4.0)
            ;

        }

        sequence
                .addTemporalMarker(() -> {
                    // robot.deposit.paintbrush.toggleFloor();
                    // robot.intake.setMotorPower(SPEED_INTAKE_STACK_APPROACH);
                    // robot.intake.setHeight(FIVE_STACK);
                    // robot.intake.setDesiredPixelCount(1);
                })

                .lineToSplineHeading(stack)
                .setTangent(LEFT)

                .addTemporalMarker(() -> {
                    // robot.intake.setMotorPower(SPEED_INTAKING);
                })
                .waitSeconds(TIME_INTAKING)
                .addTemporalMarker( () -> {
                    // robot.intake.setMotorPower(0);
                    // robot.intake.setHeight(FOUR_STACK);
                    // robot.intake.setDesiredPixelCount(2);
                })
        ;

        score(sequence, placements, 0, offset);
    }
}
