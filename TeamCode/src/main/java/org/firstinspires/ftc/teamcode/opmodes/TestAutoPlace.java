package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_FIRST;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_SECOND;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel;


@TeleOp(group = "21836 Backup")
public class TestAutoPlace extends LinearOpMode {

    public static int
                    X1 = 1,
                    Y1 = 0,
                    X2 = 5,
                    Y2 = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        double side = robot.isRed ? -1 : 1;

        Pixel[] placements = new Pixel[]{
                new Pixel(X1, Y1, EMPTY),
                new Pixel(X2, Y2, EMPTY)
        };
        Backdrop latestScan = new Backdrop();
        Pose2d scoringPos1 = placements[0].toPose2d(robot.isRed);
        Pose2d scoringPos2 = placements[1].toPose2d(robot.isRed);

        Pose2d currentPose = robot.drivetrain.getPoseEstimate();
        Pose2d startPose = new Pose2d(currentPose.getX() + (double) 0, currentPose.getY() + side * (double) 0, currentPose.getHeading());

        TrajectorySequence scoringTrajectory = robot.drivetrain.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .addTemporalMarker(() -> robot.deposit.lift.setTargetRow(placements[0].y))
                .splineTo(scoringPos1.vec(), scoringPos1.getHeading())
                .addTemporalMarker(() -> {
                    robot.deposit.paintbrush.dropPixels(1);
                    latestScan.add(placements[0]);
                })
                .waitSeconds(TIME_DROP_FIRST)
                .addTemporalMarker(() -> robot.deposit.lift.setTargetRow(placements[1].y))
                .lineTo(scoringPos2.vec())
                .addTemporalMarker(() -> {
                    robot.deposit.paintbrush.dropPixels(2);
                    latestScan.add(placements[1]);
                })
                .waitSeconds(TIME_DROP_SECOND)
                .build()
        ;

        robot.drivetrain.followTrajectorySequenceAsync(scoringTrajectory);

        waitForStart();

        while (opModeIsActive()) {
            robot.drivetrain.update();
            robot.run();
        }
    }
}
