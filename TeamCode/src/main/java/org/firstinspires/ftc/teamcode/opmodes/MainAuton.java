package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRed;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRight;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop;

@Config
@Autonomous(preselectTeleOp = "MainTeleOp")
public final class MainAuton extends LinearOpMode {

    // Declare objects:
    public static GamepadEx gamepadEx1, gamepadEx2;
    public static MultipleTelemetry mTelemetry;
    static Robot robot;
    public static Backdrop autonBackdrop = new Backdrop();
    static Pose2d autonEndPose = new Pose2d(0, 0, toRadians(90));

    public static boolean keyPressed(int gamepad, GamepadKeys.Button button) {
        return (gamepad == 2 ? gamepadEx2 : gamepadEx1).wasJustPressed(button);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry);

        // Initialize robot:
        robot = new Robot(hardwareMap);

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        // Get gamepad 1 button input and save alliance and side for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (keyPressed(1, DPAD_RIGHT))  isRight = true;
            if (keyPressed(1, DPAD_LEFT))   isRight = false;
            if (keyPressed(1, B))           isRed = true;
            if (keyPressed(1, X))           isRed = false;
            mTelemetry.addLine("Selected " + (isRed ? "RED" : "BLUE") + " " + (isRight ? "RIGHT" : "LEFT"));
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }
        mTelemetry.addLine("Confirmed " + (isRed ? "RED" : "BLUE") + " " + (isRight ? "RIGHT" : "LEFT"));
        mTelemetry.update();

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            // Manually clear old sensor data from the last loop:
            robot.readSensors();

            robot.drivetrain.update();
            robot.run();

            autonEndPose = robot.drivetrain.getPoseEstimate();
            // Push telemetry data
            robot.printTelemetry();
            mTelemetry.update();
        }
    }
}
