package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.AMBER;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.RED;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;
import org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator;


@TeleOp(group = "Single mechanism test")
public final class TestIndicators extends LinearOpMode {

    LEDIndicator[] indicators;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);
        BulkReader bulkReader = new BulkReader(hardwareMap);


        indicators = new LEDIndicator[]{
                new LEDIndicator(hardwareMap, "led right"),
                new LEDIndicator(hardwareMap, "led left")
        };

        waitForStart();

        while (opModeIsActive()) {
            bulkReader.bulkRead();

            for (LEDIndicator indicator : indicators) {
                if (keyPressed(1, B)) indicator.setState(RED);
                if (keyPressed(1, A)) indicator.setState(GREEN);
                if (keyPressed(1, X)) indicator.setState(OFF);
                if (keyPressed(1, Y)) indicator.setState(AMBER);
            }
        }
    }
}
