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
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.autonEndPose;
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
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;

@TeleOp
public final class MainTeleOp extends LinearOpMode {

    public static double loopTime;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime loopTimer = new ElapsedTime();

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize robot:
        robot = new Robot(hardwareMap);
        robot.drivetrain.setPoseEstimate(autonEndPose);
        robot.drivetrain.setCurrentHeading(autonEndPose.getHeading() - toRadians(90));

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        boolean slowModeLocked = false;

        // Get gamepad 1 button input and locks slow mode:
        while (opModeInInit()) {
            gamepadEx1.readButtons();
            if (keyPressed(1, RIGHT_BUMPER))   slowModeLocked = !slowModeLocked;
            mTelemetry.addLine((slowModeLocked ? "SLOW" : "NORMAL") + " mode");
            mTelemetry.update();
        }
        if (slowModeLocked) robot.drivetrain.lockSlowMode();

        // Control loop:
        while (opModeIsActive()) {
            loopTimer.reset();
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            robot.intake.setMotorPower(
                    gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER)
            );

            robot.deposit.lift.setLiftPower(gamepadEx2.getLeftY());

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

            double x = gamepadEx1.getRightX();
            if (gamepadEx1.isDown(LEFT_BUMPER)) {
                double y = gamepadEx1.getRightY();
                if (hypot(x, y) >= 0.8) robot.drivetrain.setCurrentHeading(atan2(y, x));
                x = 0;
            }

            // Field-centric driving with control stick inputs:
            robot.drivetrain.run(
                    gamepadEx1.getLeftX(),
                    gamepadEx1.getLeftY(),
                    x,
                    gamepadEx1.isDown(RIGHT_BUMPER) // drives slower when right shoulder button held
            );

            robot.run();

            mTelemetry.addData("full loop time", loopTime = loopTimer.seconds());
            robot.printTelemetry();
            mTelemetry.update();
        }
        robot.interrupt();
    }
}
