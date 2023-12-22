package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_FIRST;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_SECOND;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicatorGroup.State.AMBER;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicatorGroup.State.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicatorGroup.State.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicatorGroup.State.RED;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicatorGroup;


@TeleOp(group = "21836 B")
public class TestIndicators extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);

        LEDIndicatorGroup indicators = new LEDIndicatorGroup(hardwareMap, "led left", "led right");

        waitForStart();

        while (opModeIsActive()) {
            if (keyPressed(1, B)) indicators.setState(RED);
            if (keyPressed(1, A)) indicators.setState(GREEN);
            if (keyPressed(1, X)) indicators.setState(OFF);
            if (keyPressed(1, Y)) indicators.setState(AMBER);

            indicators.run();
        }
    }
}
