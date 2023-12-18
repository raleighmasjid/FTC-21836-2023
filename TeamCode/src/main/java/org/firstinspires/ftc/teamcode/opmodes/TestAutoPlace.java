package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_FIRST;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_SECOND;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.RUNNING_OFFSET_X;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.RUNNING_OFFSET_Y;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel;


@TeleOp(group = "21836 Backup")
public class TestAutoPlace extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        double side = robot.isRed ? -1 : 1;

        Pixel pixel1 = new Pixel(1, 0, EMPTY);
        Pixel pixel2 = new Pixel(5, 0, EMPTY);
        Pose2d scoringPose1 = pixel1.toPose2d(robot.isRed);
        Pose2d scoringPose2 = pixel2.toPose2d(robot.isRed);

        Pose2d currentPose = robot.drivetrain.getPoseEstimate();
        Pose2d startPose = new Pose2d(currentPose.getX() + RUNNING_OFFSET_X, currentPose.getY() + side * RUNNING_OFFSET_Y, currentPose.getHeading());

        TrajectorySequence scoringTrajectory = robot.drivetrain.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(scoringPose1.vec(), scoringPose1.getHeading())
                .addTemporalMarker(() -> robot.deposit.paintbrush.dropPixels(1))
                .waitSeconds(TIME_DROP_FIRST)
                .lineTo(scoringPose2.vec())
                .addTemporalMarker(() -> robot.deposit.paintbrush.dropPixels(2))
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
