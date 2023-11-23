package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.SimpleServoPivot.getAxonMini;
import static org.firstinspires.ftc.teamcode.subsystems.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.SimpleServoPivot.getReversedServo;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Deposit {

    public static double
            CLAW_OPEN = 8,
            CLAW_CLOSED = 0,
            HOOK_OFFSET = 0,
            PIVOT_OFFSET = 0;

    private final SimpleServoPivot pivot, hook, claw;

    private boolean floorScoring = false;
    private int pixelsLocked = 0;

    public Deposit(HardwareMap hardwareMap) {
        pivot = new SimpleServoPivot(
                new SimpleServo[]{
                        getAxonMini(hardwareMap, "deposit left"),
                        getReversedServo(getAxonMini(hardwareMap, "deposit right")),
                },
                PIVOT_OFFSET,
                PIVOT_OFFSET + 120
        );

        hook = new SimpleServoPivot(
                new SimpleServo[]{getGoBildaServo(hardwareMap, "pixel hook")},
                HOOK_OFFSET,
                HOOK_OFFSET + 180
        );

        claw = new SimpleServoPivot(
                new SimpleServo[]{getGoBildaServo(hardwareMap, "pixel claw")},
                CLAW_OPEN,
                CLAW_CLOSED
        );
    }

    public void setFloorScoring(boolean floorScoring) {
        this.floorScoring = floorScoring;
    }

    public void extend() {
        pivot.setActivated(true);
    }

    public void lockPixels() {
        pixelsLocked = 2;
    }

    public void dropPixel() {
        if (pixelsLocked == 2) dropFirstPixel(); else dropSecondPixel();
    }

    public void dropFirstPixel() {
        pixelsLocked = 1;
    }

    public void dropSecondPixel() {
        pixelsLocked = 0;
    }

    public boolean noPixels() {
        return pixelsLocked == 0;
    }

    public void retract() {
        pivot.setActivated(false);
    }

    public void run() {
        pivot.updateAngles(PIVOT_OFFSET, PIVOT_OFFSET + (floorScoring ? 170.5 : 120));
        claw.updateAngles(CLAW_OPEN, CLAW_CLOSED);
        claw.setActivated(pixelsLocked >= 1);
        hook.setActivated(pixelsLocked == 2);
        pivot.run();
        hook.run();
        claw.run();
    }
}
