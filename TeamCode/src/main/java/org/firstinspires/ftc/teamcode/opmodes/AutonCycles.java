package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.SPEED_INTAKING;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.TIME_INTAKING;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.X_INTAKING;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.X_SHIFT_CENTER_AUDIENCE_STACK_CLEARANCE;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.X_SHIFT_INTAKING;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.X_START_LEFT;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.Y_INTAKING_1;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.Y_INTAKING_2;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.Y_INTAKING_3;
import static org.firstinspires.ftc.teamcode.opmodes.AutonVars.postOuterAudience;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.LEFT;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.RIGHT;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.autonBackdrop;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.toPose2d;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_FIRST;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_SECOND;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake;

import java.util.ArrayList;

class AutonCycles {

    static Pose2d stackPos(int stack) {
        return new EditablePose(X_INTAKING, stack == 3 ? Y_INTAKING_3 : stack == 2 ? Y_INTAKING_2 : Y_INTAKING_1, LEFT).byAlliance().toPose2d();
    }

    static void driveToStack1(TrajectorySequenceBuilder sequence, Intake.Height height) {
        sequence
                .addTemporalMarker(() -> {
                    robot.intake.setHeight(height);
                })
                .setTangent(LEFT)
                .splineTo(AutonVars.enteringBackstage.byAlliance().toPose2d().vec(), LEFT)
                .splineTo(stackPos(1).vec(), LEFT)
        ;
    }

    static void driveToStack2(TrajectorySequenceBuilder sequence, Intake.Height height) {
        Pose2d turnToStack1 = new EditablePose(X_START_LEFT + X_SHIFT_CENTER_AUDIENCE_STACK_CLEARANCE, Y_INTAKING_1, LEFT).byAlliance().toPose2d();
        sequence
                .addTemporalMarker(() -> {
                    robot.intake.setHeight(height);
                })
                .setTangent(LEFT)
                .splineToConstantHeading(AutonVars.enteringBackstage.byAlliance().toPose2d().vec(), LEFT)
                .splineTo(turnToStack1.vec(), LEFT)
                .lineTo(postOuterAudience.byAlliance().toPose2d().vec())
                .lineTo(stackPos(2).vec())
        ;
    }

    static void intake2Pixels(TrajectorySequenceBuilder sequence, Intake.Height height) {
        sequence
                .addTemporalMarker(() -> {
                    robot.intake.setMotorPower(SPEED_INTAKING);
                })
                .waitSeconds(TIME_INTAKING)
                .addTemporalMarker( () -> {
                    robot.intake.setMotorPower(0);
                })
                .back(X_SHIFT_INTAKING)
                .addTemporalMarker(() -> {
                    robot.intake.setHeight(height.minus(1));
                })
                .forward(X_SHIFT_INTAKING)
                .addTemporalMarker(() -> {
                    robot.intake.setMotorPower(SPEED_INTAKING);
                })
                .waitSeconds(TIME_INTAKING)
                .addTemporalMarker( () -> {
                    robot.intake.setMotorPower(0);
                })
        ;
    }

    static void score(TrajectorySequenceBuilder sequence, ArrayList<Pixel> placements, int index) {
        score(sequence, placements, index, new Pose2d());
    }
    static void score(TrajectorySequenceBuilder sequence, ArrayList<Pixel> placements, int index, Pose2d offset) {
        Pixel first = placements.get(index);
        Pixel second = placements.get(index + 1);
        Pose2d backstage = AutonVars.enteringBackstage.byAlliance().toPose2d();

        Pose2d firstPose = toPose2d(first);
        Pose2d secondPose = toPose2d(second);
        Vector2d firstVec = firstPose.vec();
        Vector2d secondVec = secondPose.vec();

        mTelemetry.addLine(backstage.toString());
        mTelemetry.addLine(first.toString());
        mTelemetry.addLine(firstPose.toString());
        mTelemetry.addLine(firstVec.toString());
        mTelemetry.addLine(second.toString());
        mTelemetry.addLine(secondPose.toString());
        mTelemetry.addLine(secondVec.toString());
        mTelemetry.addLine();

        sequence
                .setTangent(RIGHT)
                .splineToSplineHeading(backstage, RIGHT)

                .addTemporalMarker(() -> {
//                    offsetLocalization(backstage, offset);
                    robot.deposit.lift.setTargetRow(first.y);
                })
                .splineToSplineHeading(firstPose, RIGHT)
                .addTemporalMarker(() -> {
                    robot.deposit.paintbrush.dropPixel();
                    autonBackdrop.add(first);
                })
                .waitSeconds(TIME_DROP_FIRST)

                .addTemporalMarker(() -> {
                    robot.deposit.lift.setTargetRow(second.y);
                })
                .lineTo(secondVec)
                .addTemporalMarker(() -> {
                    robot.deposit.paintbrush.dropPixel();
                    autonBackdrop.add(second);
                })
                .waitSeconds(TIME_DROP_SECOND)
        ;
    }

    static void offsetLocalization(Pose2d current, Pose2d offset) {
        robot.drivetrain.setPoseEstimate(current.minus(offset));
    }
}
