package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getAxonServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;

@Config
public final class Deposit {

    public static double
            ANGLE_PIVOT_OFFSET = 5,
            ANGLE_CLAW_OPEN = 0,
            ANGLE_CLAW_CLOSED = 15,
            ANGLE_HOOK_OPEN = 5,
            ANGLE_HOOK_CLOSED = 35,
            TIME_DROP = 1;

    private final SimpleServoPivot pivot, hook, claw;

    private final ElapsedTime timer = new ElapsedTime();
    private boolean retracted = true;
    private int pixelsLocked = 0;

    Deposit(HardwareMap hardwareMap) {
        pivot = new SimpleServoPivot(
                ANGLE_PIVOT_OFFSET,
                ANGLE_PIVOT_OFFSET + 120,
                getAxonServo(hardwareMap, "deposit left"),
                getReversedServo(getAxonServo(hardwareMap, "deposit right"))
        );

        hook = new SimpleServoPivot(
                ANGLE_HOOK_OPEN,
                ANGLE_HOOK_CLOSED,
                getGoBildaServo(hardwareMap, "pixel hook")
        );

        claw = new SimpleServoPivot(
                ANGLE_CLAW_OPEN,
                ANGLE_CLAW_CLOSED,
                getReversedServo(getGoBildaServo(hardwareMap, "pixel claw"))
        );
    }

    void lockPixels() {
        pixelsLocked = 2;
    }

    public void dropPixel() {
        if (pixelsLocked == 2) dropFirstPixel(); else dropSecondPixel();
    }

    void dropFirstPixel() {
        pixelsLocked = 1;
    }

    void dropSecondPixel() {
        pixelsLocked = 0;
        retracted = false;
        timer.reset();
    }

    void setExtended(boolean extended) {
        pivot.setActivated(extended);
        if (!extended) retracted = true;
    }

    boolean isExtended() {
        return pivot.getActivated();
    }

    boolean droppedBothPixels() {
        return !retracted && timer.seconds() >= TIME_DROP;
    }

    void run() {
        if (droppedBothPixels()) setExtended(false);

        pivot.updateAngles(ANGLE_PIVOT_OFFSET, ANGLE_PIVOT_OFFSET + 120);
        claw.updateAngles(ANGLE_CLAW_OPEN, ANGLE_CLAW_CLOSED);
        hook.updateAngles(ANGLE_HOOK_OPEN, ANGLE_HOOK_CLOSED);

        claw.setActivated(pixelsLocked >= 1);
        hook.setActivated(pixelsLocked == 2);

        pivot.run();
        claw.run();
        hook.run();
    }
}
