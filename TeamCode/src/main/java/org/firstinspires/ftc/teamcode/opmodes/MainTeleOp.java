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
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.Gamepad1;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.Gamepad2;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.Telemetry;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.IntakingHeight.FIVE_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.IntakingHeight.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.IntakingHeight.FOUR_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.IntakingHeight.THREE_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.IntakingHeight.TWO_STACK;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(group = "21836 Main")
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        Telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize robot if not already initialized:
        if (robot == null) robot = new Robot(hardwareMap);

        // Initialize gamepads:
        if (Gamepad1 == null) Gamepad1 = new GamepadEx(super.gamepad1);
        Gamepad2 = new GamepadEx(super.gamepad2);

        // Get gamepad 1 button input, locks slow mode, and saves "red" boolean for teleop configuration:
        while (opModeInInit()) {
            Gamepad1.readButtons();
            if (Gamepad1.wasJustPressed(RIGHT_BUMPER)) robot.drivetrain.toggleSlowModeLock();
            if (Gamepad1.wasJustPressed(B)) robot.isRed = true;
            if (Gamepad1.wasJustPressed(X)) robot.isRed = false;
            Telemetry.addLine((robot.drivetrain.isSlowModeLocked() ? "SLOW" : "NORMAL") + " mode");
            Telemetry.addLine((robot.isRed ? "RED" : "BLUE") + " alliance");
            Telemetry.update();
        }
        robot.start();

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            Gamepad1.readButtons();
            Gamepad2.readButtons();

            // Reset current heading as per these keybinds:
            if (Gamepad1.wasJustPressed(DPAD_UP)) robot.drivetrain.setCurrentHeading(0);
            if (Gamepad1.wasJustPressed(DPAD_LEFT)) robot.drivetrain.setCurrentHeading(PI/2);
            if (Gamepad1.wasJustPressed(DPAD_DOWN)) robot.drivetrain.setCurrentHeading(PI);
            if (Gamepad1.wasJustPressed(DPAD_RIGHT)) robot.drivetrain.setCurrentHeading(-PI/2);

            robot.intake.setMotorPower(Gamepad1.getTrigger(RIGHT_TRIGGER) - Gamepad1.getTrigger(LEFT_TRIGGER));

            if (Gamepad1.wasJustPressed(A)) robot.deposit.dropPixel();

            if (Gamepad2.wasJustPressed(DPAD_DOWN)) robot.lift.decrementRow();
            if (Gamepad2.wasJustPressed(DPAD_UP)) robot.lift.incrementRow();

            if (Gamepad2.wasJustPressed(Y)) robot.intake.setIntakingHeight(FIVE_STACK);
            if (Gamepad2.wasJustPressed(X)) robot.intake.setIntakingHeight(FOUR_STACK);
            if (Gamepad2.wasJustPressed(B)) robot.intake.setIntakingHeight(THREE_STACK);
            if (Gamepad2.wasJustPressed(A)) robot.intake.setIntakingHeight(TWO_STACK);
            if (Gamepad2.wasJustPressed(RIGHT_BUMPER)) robot.intake.setIntakingHeight(FLOOR);

            // Field-centric driving with control stick inputs:
            robot.drivetrain.run(
                    Gamepad1.getLeftX(),
                    Gamepad1.getLeftY(),
                    Gamepad1.getRightX(),
                    Gamepad1.isDown(RIGHT_BUMPER) // drives slower when bumper held
            );
            robot.run();

            // Push telemetry data to multiple outputs (set earlier):
            robot.printTelemetry(Telemetry);
            Telemetry.update();
        }
        robot.interrupt();
    }
}
