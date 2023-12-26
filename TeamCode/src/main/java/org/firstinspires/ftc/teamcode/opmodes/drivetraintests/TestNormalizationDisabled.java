package org.firstinspires.ftc.teamcode.opmodes.drivetraintests;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx2;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FIVE_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FOUR_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.THREE_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.TWO_STACK;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;

@TeleOp(group = "Drivetrain tests")
public final class TestNormalizationDisabled extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize robot:
        robot = new Robot(hardwareMap);
        robot.drivetrain.normalizeMotorPowers = false;

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
        if (slowModeLocked) robot.drivetrain.lockSlowMode();

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            // Reset current heading as per these keybinds:
            if (keyPressed(1, DPAD_UP))     robot.drivetrain.setCurrentHeading(0);
            if (keyPressed(1, DPAD_LEFT))   robot.drivetrain.setCurrentHeading(PI / 2);
            if (keyPressed(1, DPAD_DOWN))   robot.drivetrain.setCurrentHeading(PI);
            if (keyPressed(1, DPAD_RIGHT))  robot.drivetrain.setCurrentHeading(-PI / 2);


            robot.intake.setMotorPower(
                    gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER)
            );

            if (gamepadEx2.isDown(LEFT_BUMPER)) {
                if (keyPressed(2, Y))               robot.intake.setRequiredIntakingAmount(2);
                if (keyPressed(2, X))               robot.intake.setRequiredIntakingAmount(1);
                if (keyPressed(2, A))               robot.intake.setRequiredIntakingAmount(0);
            } else {
                if (keyPressed(2, DPAD_DOWN))       robot.deposit.lift.changeRow(-1);
                if (keyPressed(2, DPAD_UP))         robot.deposit.lift.changeRow(1);

                if (keyPressed(2, Y))               robot.intake.setHeight(FIVE_STACK);
                if (keyPressed(2, X))               robot.intake.setHeight(FOUR_STACK);
                if (keyPressed(2, B))               robot.intake.setHeight(THREE_STACK);
                if (keyPressed(2, A))               robot.intake.setHeight(TWO_STACK);
                if (keyPressed(2, RIGHT_BUMPER))    robot.intake.setHeight(FLOOR);

                if (keyPressed(2, DPAD_LEFT) || keyPressed(2, DPAD_RIGHT)) {
                    robot.deposit.paintbrush.dropPixels(1);
                }
            }

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
