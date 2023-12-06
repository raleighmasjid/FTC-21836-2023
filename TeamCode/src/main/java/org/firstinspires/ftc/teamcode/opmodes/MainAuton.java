package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;

@Config
@Autonomous(group = "21836 Main", preselectTeleOp = "MainTeleOp")
public final class MainAuton extends LinearOpMode {

    // Declare objects:
    static GamepadEx gamepadEx1 = null, gamepadEx2;
    static MultipleTelemetry mTelemetry;
    static Robot robot = null;

    static boolean keyPressed(int gamepad, GamepadKeys.Button button) {
        return (gamepad == 2 ? gamepadEx2 : gamepadEx1).wasJustPressed(button);
    }

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
            if (keyPressed(1, DPAD_RIGHT))     isRight = true;
            if (keyPressed(1, DPAD_LEFT))      isRight = false;
            if (keyPressed(1, B))              robot.isRed = true;
            if (keyPressed(1, X))              robot.isRed = false;
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
