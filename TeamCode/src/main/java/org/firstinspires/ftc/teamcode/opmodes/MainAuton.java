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

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@Autonomous(group = "21836 Main", preselectTeleOp = "MainTeleOp")
public final class MainAuton extends LinearOpMode {

    // Declare objects:
    static GamepadEx Gamepad1, Gamepad2;
    static MultipleTelemetry Telemetry;
    static Robot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        Telemetry = new MultipleTelemetry(telemetry);

        // Initialize robot if not already initialized:
        robot = new Robot(hardwareMap);

        // Initialize gamepad:
        Gamepad1 = new GamepadEx(super.gamepad1);

        // Get gamepad 1 button input and save alliance and side for autonomous configuration:
        boolean isRight = true;
        while (opModeInInit() && !(Gamepad1.isDown(RIGHT_BUMPER) && Gamepad1.isDown(LEFT_BUMPER))) {
            Gamepad1.readButtons();
            if (Gamepad1.wasJustPressed(DPAD_RIGHT)) isRight = true;
            if (Gamepad1.wasJustPressed(DPAD_LEFT)) isRight = false;
            if (Gamepad1.wasJustPressed(B)) robot.isRed = true;
            if (Gamepad1.wasJustPressed(X)) robot.isRed = false;
            Telemetry.addLine("Selected " + (robot.isRed ? "RED" : "BLUE") + " " + (isRight ? "RIGHT" : "LEFT"));
            Telemetry.addLine("Press both shoulder buttons to confirm!");
            Telemetry.update();
        }
        Telemetry.addLine("Confirmed " + (robot.isRed ? "RED" : "BLUE") + " " + (isRight ? "RIGHT" : "LEFT"));
        Telemetry.update();

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            // Manually clear old sensor data from the last loop:
            robot.readSensors();

            robot.drivetrain.update();
            robot.run();

            // Push telemetry data
            robot.printTelemetry(Telemetry);
            Telemetry.update();
        }
    }
}
