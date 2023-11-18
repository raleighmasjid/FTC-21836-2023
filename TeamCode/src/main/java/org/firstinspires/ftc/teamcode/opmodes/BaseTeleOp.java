package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.drivetrains.AutoTurnMecanum;

import java.util.List;

public abstract class BaseTeleOp extends LinearOpMode {

    // Declare objects:
    MultipleTelemetry myTelemetry;
    List<LynxModule> hubs;
    AutoTurnMecanum drivetrain;
    GamepadEx Gamepad1, Gamepad2;

    public void runOpMode(boolean isRed) throws InterruptedException {

        // Initialize multiple telemetry outputs:
        myTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize internal hub representations:
        // Switch hubs to manually reset sensor inputs when we tell it to:
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        drivetrain = new AutoTurnMecanum(hardwareMap);

        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);
        boolean lockSlowMode = false;

//        waitForStart();
        while (!isStarted() && !isStopRequested()) {
            if (Gamepad1.getTrigger(RIGHT_TRIGGER) > 0.5) lockSlowMode = true;
        }
        drivetrain.imu.start();

        // Control loop:
        while (opModeIsActive()) {
            // Manually clear old sensor data from the last loop:
            for (LynxModule hub : hubs) hub.clearBulkCache();
            // Read sensors + gamepads:
            Gamepad1.readButtons();
            Gamepad2.readButtons();
            drivetrain.updateGains();

            // Reset current heading as per these keybindings:
            if (Gamepad1.wasJustPressed(DPAD_UP)) drivetrain.setCurrentHeading(0);
            else if (Gamepad1.wasJustPressed(DPAD_LEFT)) drivetrain.setCurrentHeading(PI/2);
            else if (Gamepad1.wasJustPressed(DPAD_DOWN)) drivetrain.setCurrentHeading(PI);
            else if (Gamepad1.wasJustPressed(DPAD_RIGHT)) drivetrain.setCurrentHeading(-PI/2);

            // Field-centric drive dt with control stick inputs:
            drivetrain.run(
                    Gamepad1.getLeftX(),
                    Gamepad1.getLeftY(),
                    Gamepad1.getRightX(),
                    lockSlowMode ? 1 : Gamepad1.getTrigger(RIGHT_TRIGGER)
            );

            // Push telemetry data to multiple outputs (set earlier):
            drivetrain.printNumericalTelemetry(myTelemetry);
            myTelemetry.update();
        }
        drivetrain.imu.interrupt();
    }
}
