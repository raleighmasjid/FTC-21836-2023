package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx2;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.teleOpControls;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.teleOpInit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public final class AutomatedTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        boolean autoScoring = false;
        teleOpInit(this);

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            if (autoScoring) {

                if (keyPressed(1, X)) robot.drivetrain.breakFollowing();
                if (!robot.drivetrain.isBusy()) autoScoring = false;

                robot.drivetrain.update();

            } else {

                if (keyPressed(1, X)) robot.startAutoDrive();
                if (robot.beginUpdatingRunner()) autoScoring = true;

                if (gamepadEx2.isDown(LEFT_BUMPER) && gamepadEx2.isDown(RIGHT_BUMPER)) robot.scanner.reset();
                teleOpControls();
            }

            robot.run();

            mTelemetry.addData("Scoring mode", autoScoring ? "auto" : "manual");
            mTelemetry.addLine();
            robot.printTelemetry();
            mTelemetry.update();
        }
        robot.scanner.stop();
    }
}
