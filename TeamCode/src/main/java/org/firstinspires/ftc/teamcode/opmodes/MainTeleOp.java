package org.firstinspires.ftc.teamcode.opmodes;

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
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.IntakingHeight.FIVE_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.IntakingHeight.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.IntakingHeight.FOUR_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.IntakingHeight.THREE_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.IntakingHeight.TWO_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.PURPLE;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.WHITE;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.YELLOW;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.BackdropGUI;

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

        BackdropGUI gui = new BackdropGUI();

        // Get gamepad 1 button input, locks slow mode, and saves "red" boolean for teleop configuration:
        while (opModeInInit()) {
            gamepadEx1.readButtons();
            if (pressed(1, RIGHT_BUMPER)) robot.drivetrain.toggleSlowModeLock();
            if (pressed(1, B)) robot.isRed = true;
            if (pressed(1, X)) robot.isRed = false;
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
            if (pressed(1, DPAD_UP))            robot.drivetrain.setCurrentHeading(0);
            if (pressed(1, DPAD_LEFT))          robot.drivetrain.setCurrentHeading(PI / 2);
            if (pressed(1, DPAD_DOWN))          robot.drivetrain.setCurrentHeading(PI);
            if (pressed(1, DPAD_RIGHT))         robot.drivetrain.setCurrentHeading(-PI / 2);

            if (pressed(1, A))                  robot.deposit.dropPixel();

            robot.intake.setMotorPower(
                    gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER)
            );

            if (gamepadEx2.isDown(LEFT_BUMPER)) {

                if (pressed(2, DPAD_UP))        gui.up();
                if (pressed(2, DPAD_DOWN))      gui.down();
                if (pressed(2, DPAD_LEFT))      gui.left();
                if (pressed(2, DPAD_RIGHT))     gui.right();

                if (pressed(2, Y))              gui.update(YELLOW);
                if (pressed(2, X))              gui.update(PURPLE);
                if (pressed(2, A))              gui.update(GREEN);
                if (pressed(2, B))              gui.update(WHITE);
                if (pressed(2, RIGHT_BUMPER))   gui.update(EMPTY);

            } else {

                if (pressed(2, DPAD_DOWN))      robot.lift.decrementRow();
                if (pressed(2, DPAD_UP))        robot.lift.incrementRow();

                if (pressed(2, Y))              robot.intake.setHeight(FIVE_STACK);
                if (pressed(2, X))              robot.intake.setHeight(FOUR_STACK);
                if (pressed(2, B))              robot.intake.setHeight(THREE_STACK);
                if (pressed(2, A))              robot.intake.setHeight(TWO_STACK);
                if (pressed(2, RIGHT_BUMPER))   robot.intake.setHeight(FLOOR);

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
            gui.toTelemetry(mTelemetry);
            robot.printTelemetry(mTelemetry);
            mTelemetry.update();
        }
        robot.interrupt();
    }

    private static boolean pressed(int gamepad, GamepadKeys.Button button) {
        return (gamepad == 2 ? gamepadEx2 : gamepadEx1).wasJustPressed(button);
    }
}
