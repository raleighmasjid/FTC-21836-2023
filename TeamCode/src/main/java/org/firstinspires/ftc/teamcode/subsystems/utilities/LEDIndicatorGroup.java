package org.firstinspires.ftc.teamcode.subsystems.utilities;

import static com.qualcomm.robotcore.hardware.DigitalChannel.Mode.OUTPUT;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicatorGroup.State.AMBER;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicatorGroup.State.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicatorGroup.State.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.LEDIndicatorGroup.State.RED;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LEDIndicatorGroup {

    public enum State {
        RED,
        GREEN,
        AMBER,
        OFF,
    }

    private State state = OFF;
    private final DigitalChannel[][] indicators;

    public LEDIndicatorGroup(HardwareMap hardwareMap, int indicatorCount) {
        indicators = new DigitalChannel[indicatorCount][2];
        for (int i = 0; i < indicatorCount; i++) {
            indicators[i][0] = hardwareMap.get(DigitalChannel.class, "led " + (i + 1) + " red");
            indicators[i][1] = hardwareMap.get(DigitalChannel.class, "led " + (i + 1) + " green");
            indicators[i][0].setMode(OUTPUT);
            indicators[i][1].setMode(OUTPUT);
        }
    }

    public void setState(State state) {
        this.state = state;
    }

    public void run() {
        for (DigitalChannel[] indicator : indicators) {
            indicator[0].setState(state == RED || state == AMBER);
            indicator[1].setState(state == GREEN || state == AMBER);
        }
    }
}
