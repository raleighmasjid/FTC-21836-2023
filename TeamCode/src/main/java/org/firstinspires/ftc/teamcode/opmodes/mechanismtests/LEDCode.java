package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp

public class LEDCode extends LinearOpMode {
    DigitalChannel touch;
    DigitalChannel redLED;
    DigitalChannel greenLED;


    @Override
    public void runOpMode() {
        // Get the LED colors and touch sensor from the hardwaremap
        redLED = hardwareMap.get(DigitalChannel.class, "led left red");
        greenLED = hardwareMap.get(DigitalChannel.class, "led left green");

        // Wait for the play button to be pressed
        waitForStart();

        // change LED mode from input to output
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        // Loop while the Op Mode is running
        while (opModeIsActive()) {
            if (gamepad1.a) {
                greenLED.setState(true);
                redLED.setState(false);
            }
            if (gamepad1.b) {
                greenLED.setState(false);
                redLED.setState(true);
            }
        }
    }
}