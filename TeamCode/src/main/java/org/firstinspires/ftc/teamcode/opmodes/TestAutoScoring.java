package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.BackdropScanner;


@TeleOp(group = "21836 B")
public final class TestAutoScoring extends LinearOpMode {

    public static int
                    X1 = 1,
                    Y1 = 0,
                    X2 = 5,
                    Y2 = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        BackdropScanner scanner = new BackdropScanner(robot);

        robot.drivetrain.followTrajectorySequenceAsync(scanner.getScoringTrajectory());

        waitForStart();

        while (opModeIsActive()) {
            robot.readSensors();
            robot.drivetrain.update();
            robot.run();
        }
    }
}
