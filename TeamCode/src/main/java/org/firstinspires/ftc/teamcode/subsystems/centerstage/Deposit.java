package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getAxonServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;

public final class Deposit {

    public static double
            ANGLE_CLAW_OPEN = 8,
            ANGLE_CLAW_CLOSED = 0,
            ANGLE_OFFSET_HOOK = 0,
            ANGLE_OFFSET_PIVOT = 0,
            TIME_DROP = 0.25;

    private final SimpleServoPivot pivot, hook, claw;

    private final ElapsedTime timer = new ElapsedTime();
    private boolean retracted = true;
    private int pixelsLocked = 0;

    public Deposit(HardwareMap hardwareMap) {
        pivot = new SimpleServoPivot(
                ANGLE_OFFSET_PIVOT,
                ANGLE_OFFSET_PIVOT + 120,
                getAxonServo(hardwareMap, "deposit left"),
                getReversedServo(getAxonServo(hardwareMap, "deposit right"))
        );

        hook = new SimpleServoPivot(
                ANGLE_OFFSET_HOOK,
                ANGLE_OFFSET_HOOK + 180,
                getGoBildaServo(hardwareMap, "pixel hook")
        );

        claw = new SimpleServoPivot(
                ANGLE_CLAW_OPEN,
                ANGLE_CLAW_CLOSED,
                getGoBildaServo(hardwareMap, "pixel claw")
        );
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
        retracted = false;
        timer.reset();
    }

    public void extend() {
        pivot.setActivated(true);
    }

    public void retract() {
        pivot.setActivated(false);
        retracted = true;
    }

    public boolean isExtended() {
        return pivot.getActivated();
    }

    public boolean droppedBothPixels() {
        return !retracted && timer.seconds() >= TIME_DROP;
    }

    public void run() {
        if (droppedBothPixels()) retract();

        pivot.updateAngles(ANGLE_OFFSET_PIVOT, ANGLE_OFFSET_PIVOT + 120);
        claw.updateAngles(ANGLE_CLAW_OPEN, ANGLE_CLAW_CLOSED);
        hook.updateAngles(ANGLE_OFFSET_HOOK, ANGLE_OFFSET_HOOK + 180);

        claw.setActivated(pixelsLocked >= 1);
        hook.setActivated(pixelsLocked == 2);

        pivot.run();
        claw.run();
        hook.run();
    }
}
