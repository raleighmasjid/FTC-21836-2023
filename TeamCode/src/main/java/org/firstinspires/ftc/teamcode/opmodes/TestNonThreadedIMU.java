package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx2;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drivetrains.RawIMUMecanumDrivetrain;

@TeleOp(group = "Drivetrain tests")
public final class TestNonThreadedIMU extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize robot:
        RawIMUMecanumDrivetrain drivetrain = new RawIMUMecanumDrivetrain(hardwareMap);

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        boolean slowModeLocked = false;

        // Get gamepad 1 button input, locks slow mode, and saves "red" boolean for teleop configuration:
        while (opModeInInit()) {
            gamepadEx1.readButtons();
            if (keyPressed(1, RIGHT_BUMPER))   slowModeLocked = !slowModeLocked;
            mTelemetry.addLine((slowModeLocked ? "SLOW" : "NORMAL") + " mode");
            mTelemetry.update();
        }
        if (slowModeLocked) drivetrain.lockSlowMode();

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            drivetrain.readIMU();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            // Reset current heading as per these keybinds:
            if (keyPressed(1, DPAD_UP))     drivetrain.setCurrentHeading(0);
            if (keyPressed(1, DPAD_LEFT))   drivetrain.setCurrentHeading(PI / 2);
            if (keyPressed(1, DPAD_DOWN))   drivetrain.setCurrentHeading(PI);
            if (keyPressed(1, DPAD_RIGHT))  drivetrain.setCurrentHeading(-PI / 2);

            // Field-centric driving with control stick inputs:
            drivetrain.run(
                    gamepadEx1.getLeftX(),
                    gamepadEx1.getLeftY(),
                    gamepadEx1.getRightX(),
                    gamepadEx1.isDown(RIGHT_BUMPER) // drives slower when right shoulder button held
            );

            drivetrain.printNumericalTelemetry(mTelemetry);
            mTelemetry.update();
        }
    }
}
