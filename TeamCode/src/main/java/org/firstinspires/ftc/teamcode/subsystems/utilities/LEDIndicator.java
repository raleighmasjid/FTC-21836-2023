package org.firstinspires.ftc.teamcode.subsystems.utilities;

import static com.qualcomm.robotcore.hardware.DigitalChannel.Mode.OUTPUT;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.AMBER;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicator.State.OFF;
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

    private State state = OFF;
    private final DigitalChannel green, red;

    public LEDIndicator(HardwareMap hardwareMap, String name) {
        green = hardwareMap.get(DigitalChannel.class, name + " green");
        red = hardwareMap.get(DigitalChannel.class, name + " red");
        green.setMode(OUTPUT);
        red.setMode(OUTPUT);
    }

    public void setState(State state) {
        this.state = state;
    }

    public void run() {
        green.setState(state == GREEN || state == AMBER);
        red.setState(state == RED || state == AMBER);
    }
}
