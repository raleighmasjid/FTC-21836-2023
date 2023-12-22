package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_FIRST;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_SECOND;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.WHITE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel;


@TeleOp(group = "21836 B")
public class TestAutoScoring extends LinearOpMode {

    public static int
                    X1 = 1,
                    Y1 = 0,
                    X2 = 5,
                    Y2 = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        Pixel[] placements = new Pixel[]{
                new Pixel(X1, Y1, WHITE),
                new Pixel(X2, Y2, WHITE)
        };
        Backdrop latestScan = new Backdrop();

        Pose2d scoringPos1 = placements[0].toPose2d();
        Pose2d scoringPos2 = placements[1].toPose2d();

        Pose2d startPose = robot.drivetrain.getPoseEstimate();

        boolean sameScoringLocation = scoringPos1.epsilonEquals(scoringPos2);

        TrajectorySequence scoringTrajectory =
                sameScoringLocation ?
                                robot.drivetrain.trajectorySequenceBuilder(startPose)
                                        .addTemporalMarker(() -> {
                                            robot.deposit.lift.setTargetRow(placements[1].y);
                                        })
                                        .lineToSplineHeading(scoringPos2)
                                        .addTemporalMarker(() -> {
                                            robot.deposit.paintbrush.dropPixels(2);
                                            latestScan.add(placements[1]);
                                        })
                                        .waitSeconds(TIME_DROP_SECOND)
                                        .addTemporalMarker(() -> {
//                                            trajectoryReady = false;
                                        })
                                        .build() :
                                robot.drivetrain.trajectorySequenceBuilder(startPose)
                                        .addTemporalMarker(() -> {
                                            robot.deposit.lift.setTargetRow(placements[0].y);
                                        })
                                        .lineToSplineHeading(scoringPos1)
                                        .addTemporalMarker(() -> {
                                            robot.deposit.paintbrush.dropPixels(1);
                                            latestScan.add(placements[0]);
                                        })
                                        .waitSeconds(TIME_DROP_FIRST)
                                        .addTemporalMarker(() -> {
                                            robot.deposit.lift.setTargetRow(placements[1].y);
                                        })
                                        .lineToConstantHeading(scoringPos2.vec())
                                        .addTemporalMarker(() -> {
                                            robot.deposit.paintbrush.dropPixels(2);
                                            latestScan.add(placements[1]);
                                        })
                                        .waitSeconds(TIME_DROP_SECOND)
                                        .addTemporalMarker(() -> {
//                                            trajectoryReady = false;
                                        })
                                        .build()
        ;

        robot.drivetrain.followTrajectorySequenceAsync(scoringTrajectory);

        waitForStart();

        while (opModeIsActive()) {
            robot.readSensors();
            robot.drivetrain.update();
            robot.run();
        }
    }
}
