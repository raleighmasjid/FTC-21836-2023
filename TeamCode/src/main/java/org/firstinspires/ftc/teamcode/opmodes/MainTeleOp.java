package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.*;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.red;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drivetrains.AutoTurnMecanum;

import java.util.List;

@TeleOp(group = "21836 TeleOp")
public class MainTeleOp extends LinearOpMode {

    // Declare objects:
    MultipleTelemetry myTelemetry;
    List<LynxModule> hubs;
    AutoTurnMecanum drivetrain;
    GamepadEx Gamepad1, Gamepad2;

    @Override
    public void runOpMode() throws InterruptedException {

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

        while (!isStarted() && !isStopRequested()) {
            Gamepad1.readButtons();
            if (Gamepad1.wasJustPressed(RIGHT_BUMPER)) lockSlowMode = !lockSlowMode;
            if (Gamepad1.wasJustPressed(B)) red = true;
            if (Gamepad1.wasJustPressed(X)) red = false;
            myTelemetry.addLine("Slow mode " + (lockSlowMode ? "ON" : "OFF"));
            myTelemetry.addLine("Selected " + (red ? "RED" : "BLUE"));
            myTelemetry.update();
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

            // Reset current heading as per these keybinds:
            if (Gamepad1.wasJustPressed(DPAD_UP)) drivetrain.setCurrentHeading(0);
            if (Gamepad1.wasJustPressed(DPAD_LEFT)) drivetrain.setCurrentHeading(PI/2);
            if (Gamepad1.wasJustPressed(DPAD_DOWN)) drivetrain.setCurrentHeading(PI);
            if (Gamepad1.wasJustPressed(DPAD_RIGHT)) drivetrain.setCurrentHeading(-PI/2);

            if (lockSlowMode && Gamepad1.wasJustPressed(RIGHT_BUMPER)) lockSlowMode = false;
            // Field-centric drive dt with control stick inputs:
            drivetrain.run(
                    Gamepad1.getLeftX(),
                    Gamepad1.getLeftY(),
                    Gamepad1.getRightX(),
                    (lockSlowMode ? 1 : 0) + Gamepad1.getTrigger(RIGHT_TRIGGER)
            );

            // Push telemetry data to multiple outputs (set earlier):
            drivetrain.printNumericalTelemetry(myTelemetry);
            myTelemetry.update();
        }
        drivetrain.imu.interrupt();
    }
}
