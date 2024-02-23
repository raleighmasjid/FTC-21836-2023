package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.AutonCycles.stackPos;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.ANGLE_INNER_SPIKE_AUDIENCE_APPROACH;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.ANGLE_OUTER_SPIKE_APPROACH_BLUE;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.ANGLE_OUTER_SPIKE_APPROACH_RED;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.SPEED_INTAKE_STACK_APPROACH;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.SPEED_INTAKING;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.TIME_INTAKING;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.TIME_PRE_SPIKE_AUDIENCE_PAINTBRUSH;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.TIME_PRE_YELLOW;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.TIME_SPIKE_AUDIENCE;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.TIME_SPIKE_BACKDROP;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.TIME_SPIKE_TO_INTAKE_FLIP;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.X_SHIFT_PRE_STACK_AUDIENCE_INNER_SPIKE;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.Y_SHIFT_POST_INNER_SPIKE_BACKDROP;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.centerSpikeAudience;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.centerSpikeBackdrop;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.innerSpikeAudience;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.innerSpikeBackdrop;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.offsetAudienceCenter;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.offsetAudienceInner;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.offsetAudienceOuter;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.outerSpikeAudience;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.outerSpikeBackdrop;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.FORWARD;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.LEFT;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.REVERSE;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.RIGHT;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.autonBackdrop;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static org.firstinspires.ftc.teamcode.opmodes.AutonCycles.score;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.toPose2d;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_SECOND;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FIVE_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FOUR_STACK;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.isRed;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

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
                    robot.spike.toggle();
                })
                .waitSeconds(TIME_SPIKE_BACKDROP)
                .setTangent(RIGHT)
                .UNSTABLE_addTemporalMarkerOffset(TIME_SPIKE_TO_INTAKE_FLIP, () -> {
                    robot.deposit.lift.setTargetRow(placements.get(0).y);
                })
                .splineToConstantHeading(toPose2d(placements.get(0)).vec(),
                        outer ?
                                isRed ? ANGLE_OUTER_SPIKE_APPROACH_RED : ANGLE_OUTER_SPIKE_APPROACH_BLUE :
                                RIGHT
                )
                .waitSeconds(TIME_PRE_YELLOW)
                .addTemporalMarker(() -> {
                    robot.deposit.paintbrush.dropPixel();
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
                    robot.deposit.lift.setTargetRow(0);
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
                        robot.deposit.paintbrush.dropPixel();
                    })
                    .waitSeconds(TIME_SPIKE_AUDIENCE)
                    .addTemporalMarker(() -> {
                        robot.deposit.lift.setTargetRow(-1);
                        robot.intake.toggle();
                    })
                    .lineToSplineHeading(preStack.toPose2d())
            ;

        } else if (outer) {

            Pose2d spike = outerSpikeAudience.byAlliance().toPose2d();
            sequence
                    .addTemporalMarker(() -> {
                        robot.intake.toggle();
                    })
                    .splineTo(spike.vec(), spike.getHeading())
                    .addTemporalMarker(() -> {
                        robot.deposit.paintbrush.dropPixel();
                    })
                    .waitSeconds(TIME_SPIKE_AUDIENCE)
                    .addTemporalMarker(() -> {
                        robot.deposit.lift.setTargetRow(-1);
                    })
                    .setTangent(a * FORWARD)
                    .turn(a * FORWARD)
            ;

        } else {

            Pose2d spike = centerSpikeAudience.byAlliance().toPose2d();
            sequence
                    .forward(10)
                    .addTemporalMarker(() -> {
                        robot.intake.toggle();
                    })
                    .splineTo(spike.vec(), spike.getHeading())
                    .addTemporalMarker(() -> {
                        robot.deposit.paintbrush.dropPixel();
                    })
                    .waitSeconds(TIME_SPIKE_AUDIENCE)
                    .addTemporalMarker(() -> {
                        robot.deposit.lift.setTargetRow(-1);
                    })
                    .setTangent(a * FORWARD)
                    .turn(a * PI / 4.0)
            ;

        }

        sequence
                .addTemporalMarker(() -> {
                    robot.deposit.paintbrush.toggleFloor();
                    robot.intake.setMotorPower(SPEED_INTAKE_STACK_APPROACH);
                    robot.intake.setHeight(FIVE_STACK);
                    robot.intake.setDesiredPixelCount(1);
                })

                .lineToSplineHeading(stack)
                .setTangent(LEFT)

                .addTemporalMarker(() -> {
                    robot.intake.setMotorPower(SPEED_INTAKING);
                })
                .waitSeconds(TIME_INTAKING)
                .addTemporalMarker( () -> {
                    robot.intake.setMotorPower(0);
                    robot.intake.setHeight(FOUR_STACK);
                    robot.intake.setDesiredPixelCount(2);
                })
        ;

        score(sequence, placements, 0, offset);
    }
}
