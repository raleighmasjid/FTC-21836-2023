package org.firstinspires.ftc.teamcode.subsystems.utilities;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LEDIndicator {

    public enum State {
        RED,
        GREEN,
        AMBER,
        OFF,
    }

    private final DigitalChannel red, green;
    private boolean greenOn, redOn;

    public LEDIndicator(HardwareMap hardwareMap, String name) {
        // Get the LED colors and touch sensor from the hardwaremap
        green = hardwareMap.get(DigitalChannel.class, name + " green");
        red = hardwareMap.get(DigitalChannel.class, name + " red");

        // change LED mode from input to output
        red.setMode(DigitalChannel.Mode.OUTPUT);
        green.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void setState(State state) {
        greenOn = state == State.GREEN || state == State.AMBER;
        redOn = state == State.RED || state == State.AMBER;
    }

    public void run() {
        green.setState(greenOn);
        red.setState(redOn);
    }
}