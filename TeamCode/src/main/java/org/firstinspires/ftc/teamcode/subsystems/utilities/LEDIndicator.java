package org.firstinspires.ftc.teamcode.subsystems.utilities;

import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.AMBER;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.RED;

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

    public LEDIndicator(HardwareMap hardwareMap, String name) {
        // Get the LED colors and touch sensor from the hardwaremap
        green = hardwareMap.get(DigitalChannel.class, name + " green");
        red = hardwareMap.get(DigitalChannel.class, name + " red");

        // change LED mode from input to output
        red.setMode(DigitalChannel.Mode.OUTPUT);
        green.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void setState(State state) {
        green.setState(state == GREEN || state == AMBER);
        red.setState(state == RED || state == AMBER);
    }
}