package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@Autonomous(group = "21836 Main", preselectTeleOp = "MainTeleOp")
public class MainAuton extends LinearOpMode {

    // Declare objects:
    MultipleTelemetry myTelemetry;
    GamepadEx gamepad1;
    static Robot robot = null;
    static Backdrop backdrop = new Backdrop();

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        myTelemetry = new MultipleTelemetry(telemetry);

        // Initialize gamepad (ONLY FOR INIT, DON'T CALL DURING WHILE LOOP)
        gamepad1 = new GamepadEx(super.gamepad1);

        // Get gamepad 1 button input and save alliance and side for autonomous configuration:
        boolean right = true;
        while (opModeInInit() && !(gamepad1.isDown(RIGHT_BUMPER) && gamepad1.isDown(LEFT_BUMPER))) {
            gamepad1.readButtons();
            if (gamepad1.wasJustPressed(DPAD_RIGHT)) right = true;
            if (gamepad1.wasJustPressed(DPAD_LEFT)) right = false;
            if (gamepad1.wasJustPressed(B)) robot.red = true;
            if (gamepad1.wasJustPressed(X)) robot.red = false;
            myTelemetry.addLine("Selected " + (robot.red ? "RED" : "BLUE") + " " + (right ? "RIGHT" : "LEFT"));
            myTelemetry.addLine("Press both shoulder buttons to confirm!");
            myTelemetry.update();
        }
        myTelemetry.addLine("Confirmed " + (robot.red ? "RED" : "BLUE") + " " + (right ? "RIGHT" : "LEFT"));
        myTelemetry.update();

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            // Manually clear old sensor data from the last loop:
            robot.readSensors();

            // Push telemetry data
            myTelemetry.update();
        }
    }
}
