package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
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
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.PlacementCalculator.initialColors;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel;

@TeleOp
@Disabled
public final class AutomatedTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        boolean autoScoring = false;
        teleOpInit(this);
        int selectedColor = 0;

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            if (autoScoring) {

                if (keyPressed(1, X)) robot.drivetrain.breakFollowing();
                if (!robot.drivetrain.isBusy()) autoScoring = false;

            } else {

//                if (keyPressed(1, X)) autoScoring = robot.autoScore();

                if (gamepadEx2.isDown(LEFT_BUMPER)) {
                    if (gamepadEx2.isDown(RIGHT_BUMPER)) robot.autoScoringManager.reset();
                    if (keyPressed(2, DPAD_RIGHT)) selectedColor = (selectedColor + 1) % 3;
                    if (keyPressed(2, DPAD_LEFT)) selectedColor = (selectedColor - 1) % 3;
                    if (keyPressed(2, DPAD_UP)) initialColors[selectedColor]++;
                    if (keyPressed(2, DPAD_DOWN)) initialColors[selectedColor]--;
                }
                teleOpControls();
            }

            robot.run();

            mTelemetry.addData("Scoring mode", autoScoring ? "auto" : "manual");
            mTelemetry.addLine(Pixel.Color.get(selectedColor).name() + " pixels left: " + initialColors[selectedColor]);
            mTelemetry.addLine();
            robot.printTelemetry();
            mTelemetry.update();
        }
        robot.autoScoringManager.stop();
    }
}
