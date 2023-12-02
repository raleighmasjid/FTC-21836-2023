package org.firstinspires.ftc.teamcode.subsystems.utilities;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.control.gainmatrices.HSV;
import org.firstinspires.ftc.teamcode.control.gainmatrices.RGB;

public final class ThreadedColorSensor extends Thread {

    private final NormalizedColorSensor sensor;
    private NormalizedRGBA rgba = new NormalizedRGBA();
    private final float[] hsv = new float[3];
    private boolean run = true;

    public ThreadedColorSensor(HardwareMap hardwareMap, String name, float gain) {
        sensor = hardwareMap.get(NormalizedColorSensor.class, name);
        if (sensor instanceof SwitchableLight) ((SwitchableLight) sensor).enableLight(true);
        sensor.setGain(gain);

        start();
    }

    public void run() {
        while (run) rgba = sensor.getNormalizedColors();
    }

    public void interrupt() {
        run = false;
    }

    public HSV getHSV() {
        Color.colorToHSV(rgba.toColor(), hsv);
        return new HSV(
                hsv[0],
                hsv[1],
                hsv[2]
        );
    }

    public RGB getRGB() {
        return new RGB(
                rgba.red * 255,
                rgba.green * 255,
                rgba.blue * 255
        );
    }
}
