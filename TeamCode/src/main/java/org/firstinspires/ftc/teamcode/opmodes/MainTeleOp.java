package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.red;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.List;

@TeleOp(group = "21836 Main")
public class MainTeleOp extends LinearOpMode {

    // Declare objects:
    MultipleTelemetry myTelemetry;
    List<LynxModule> hubs;
    GamepadEx gamepad1, gamepad2;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        myTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize internal hub representations:
        // Switch hubs to manually reset sensor inputs when we tell it to:
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // Initialize robot if not already initialized:
        if (robot == null) robot = new Robot(hardwareMap);

        // Initialize gamepads
        gamepad1 = new GamepadEx(super.gamepad1);
        gamepad2 = new GamepadEx(super.gamepad2);

        // Get gamepad 1 button input and locks slow mode and saves "red" boolean for teleop configuration:
        while (opModeInInit()) {
            gamepad1.readButtons();
            if (gamepad1.wasJustPressed(RIGHT_BUMPER)) robot.drivetrain.toggleSlowModeLock();
            if (gamepad1.wasJustPressed(B)) red = true;
            if (gamepad1.wasJustPressed(X)) red = false;
            myTelemetry.addLine((robot.drivetrain.isSlowModeLocked() ? "SLOW" : "NORMAL") + " mode");
            myTelemetry.addLine((red ? "RED" : "BLUE") + " alliance");
            myTelemetry.update();
        }
        robot.start();

        // Control loop:
        while (opModeIsActive()) {
            // Manually clear old sensor data from the last loop:
            for (LynxModule hub : hubs) hub.clearBulkCache();
            // Read sensors + gamepads:
            gamepad1.readButtons();
            gamepad2.readButtons();
            robot.readSensors();

            // Reset current heading as per these keybinds:
            if (gamepad1.wasJustPressed(DPAD_UP)) robot.drivetrain.setCurrentHeading(0);
            if (gamepad1.wasJustPressed(DPAD_LEFT)) robot.drivetrain.setCurrentHeading(PI/2);
            if (gamepad1.wasJustPressed(DPAD_DOWN)) robot.drivetrain.setCurrentHeading(PI);
            if (gamepad1.wasJustPressed(DPAD_RIGHT)) robot.drivetrain.setCurrentHeading(-PI/2);

            // Field-centric drive dt with control stick inputs:
            robot.drivetrain.run(
                    gamepad1.getLeftX(),
                    gamepad1.getLeftY(),
                    gamepad1.getRightX(),
                    gamepad1.isDown(RIGHT_BUMPER)
            );

            // Push telemetry data to multiple outputs (set earlier):
            robot.printTelemetry(myTelemetry);
            myTelemetry.update();
        }
        robot.interrupt();
    }
}
