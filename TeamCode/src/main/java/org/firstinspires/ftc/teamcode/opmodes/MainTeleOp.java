package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx2;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.IntakingHeight.FIVE_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.IntakingHeight.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.IntakingHeight.FOUR_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.IntakingHeight.THREE_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.IntakingHeight.TWO_STACK;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;

@TeleOp(group = "21836 Main")
public final class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize robot if not already initialized:
        if (robot == null) robot = new Robot(hardwareMap);

        // Initialize gamepads:
        if (gamepadEx1 == null) gamepadEx1 = new GamepadEx(super.gamepad1);
        gamepadEx2 = new GamepadEx(super.gamepad2);

        // Get gamepad 1 button input, locks slow mode, and saves "red" boolean for teleop configuration:
        while (opModeInInit()) {
            gamepadEx1.readButtons();
            if (gamepadEx1.wasJustPressed(RIGHT_BUMPER)) robot.drivetrain.toggleSlowModeLock();
            if (gamepadEx1.wasJustPressed(B)) robot.isRed = true;
            if (gamepadEx1.wasJustPressed(X)) robot.isRed = false;
            mTelemetry.addLine((robot.drivetrain.isSlowModeLocked() ? "SLOW" : "NORMAL") + " mode");
            mTelemetry.addLine((robot.isRed ? "RED" : "BLUE") + " alliance");
            mTelemetry.update();
        }
        robot.start();

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            // Reset current heading as per these keybinds:
            if (gamepadEx1.wasJustPressed(DPAD_UP)) robot.drivetrain.setCurrentHeading(0);
            if (gamepadEx1.wasJustPressed(DPAD_LEFT)) robot.drivetrain.setCurrentHeading(PI/2);
            if (gamepadEx1.wasJustPressed(DPAD_DOWN)) robot.drivetrain.setCurrentHeading(PI);
            if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) robot.drivetrain.setCurrentHeading(-PI/2);

            robot.intake.setMotorPower(gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER));

            if (gamepadEx1.wasJustPressed(A)) robot.deposit.dropPixel();

            if (gamepadEx2.wasJustPressed(DPAD_DOWN)) robot.lift.decrementRow();
            if (gamepadEx2.wasJustPressed(DPAD_UP)) robot.lift.incrementRow();

            if (gamepadEx2.wasJustPressed(Y)) robot.intake.setIntakingHeight(FIVE_STACK);
            if (gamepadEx2.wasJustPressed(X)) robot.intake.setIntakingHeight(FOUR_STACK);
            if (gamepadEx2.wasJustPressed(B)) robot.intake.setIntakingHeight(THREE_STACK);
            if (gamepadEx2.wasJustPressed(A)) robot.intake.setIntakingHeight(TWO_STACK);
            if (gamepadEx2.wasJustPressed(RIGHT_BUMPER)) robot.intake.setIntakingHeight(FLOOR);

            // Field-centric driving with control stick inputs:
            robot.drivetrain.run(
                    gamepadEx1.getLeftX(),
                    gamepadEx1.getLeftY(),
                    gamepadEx1.getRightX(),
                    gamepadEx1.isDown(RIGHT_BUMPER) // drives slower when right shoulder button held
            );
            robot.run();

            // Push telemetry data to multiple outputs (set earlier):
            robot.printTelemetry(mTelemetry);
            mTelemetry.update();
        }
        robot.interrupt();
    }
}
