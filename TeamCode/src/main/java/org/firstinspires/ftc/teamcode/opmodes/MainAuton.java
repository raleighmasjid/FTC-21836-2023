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

import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;

@Config
@Autonomous(group = "21836 Main", preselectTeleOp = "MainTeleOp")
public final class MainAuton extends LinearOpMode {

    // Declare objects:
    static GamepadEx gamepadEx1, gamepadEx2;
    static MultipleTelemetry mTelemetry;
    static Robot robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry);

        // Initialize robot if not already initialized:
        robot = new Robot(hardwareMap);

        // Initialize gamepad:
        gamepadEx1 = new GamepadEx(super.gamepad1);

        // Get gamepad 1 button input and save alliance and side for autonomous configuration:
        boolean isRight = true;
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) isRight = true;
            if (gamepadEx1.wasJustPressed(DPAD_LEFT)) isRight = false;
            if (gamepadEx1.wasJustPressed(B)) robot.isRed = true;
            if (gamepadEx1.wasJustPressed(X)) robot.isRed = false;
            mTelemetry.addLine("Selected " + (robot.isRed ? "RED" : "BLUE") + " " + (isRight ? "RIGHT" : "LEFT"));
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }
        mTelemetry.addLine("Confirmed " + (robot.isRed ? "RED" : "BLUE") + " " + (isRight ? "RIGHT" : "LEFT"));
        mTelemetry.update();

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            // Manually clear old sensor data from the last loop:
            robot.readSensors();

            robot.drivetrain.update();
            robot.run();

            // Push telemetry data
            robot.printTelemetry(mTelemetry);
            mTelemetry.update();
        }
    }
}
