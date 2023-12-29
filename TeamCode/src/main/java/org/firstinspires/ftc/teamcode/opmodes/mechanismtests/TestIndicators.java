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
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;
import org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator;


@TeleOp(group = "Single mechanism test")
public final class TestIndicators extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);
        BulkReader bulkReader = new BulkReader(hardwareMap);

        DigitalChannel redLED = hardwareMap.get(DigitalChannel.class, "led left red");
        DigitalChannel greenLED = hardwareMap.get(DigitalChannel.class, "led left green");
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        LEDIndicator.State state = RED;

        waitForStart();

        while (opModeIsActive()) {
            bulkReader.bulkRead();

            if (keyPressed(1, B)) state = RED;
            if (keyPressed(1, A)) state = GREEN;
            if (keyPressed(1, X)) state = OFF;
            if (keyPressed(1, Y)) state = AMBER;
            redLED.setState(state == RED || state == AMBER);
            greenLED.setState(state == GREEN || state == AMBER);
        }
    }
}
