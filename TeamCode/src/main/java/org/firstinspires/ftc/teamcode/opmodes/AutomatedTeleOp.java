package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.BACKWARD;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.FORWARD;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.autonEndPose;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx2;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.teleOpControls;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;

@TeleOp
public final class AutomatedTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize robot:
        robot = new Robot(hardwareMap);
        robot.drivetrain.setPoseEstimate(autonEndPose);
        robot.drivetrain.setCurrentHeading(autonEndPose.getHeading() - (isRed ? FORWARD : BACKWARD));
        robot.algorithmInit();

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        boolean autoScoring = false;
        boolean slowModeLocked = false;

        // Get gamepad 1 button input, locks slow mode, and saves "red" boolean for teleop configuration:
        while (opModeInInit()) {
            gamepadEx1.readButtons();
            if (keyPressed(1, RIGHT_BUMPER))   slowModeLocked = !slowModeLocked;
            if (keyPressed(1, B))              isRed = true;
            if (keyPressed(1, X))              isRed = false;
            mTelemetry.addLine((slowModeLocked ? "SLOW" : "NORMAL") + " mode");
            mTelemetry.addLine((isRed ? "RED" : "BLUE") + " alliance");
            mTelemetry.update();
        }
        if (slowModeLocked) robot.drivetrain.lockSlowMode();

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            if (robot.beginUpdatingRunner()) autoScoring = true;

            if (autoScoring) {

                if (keyPressed(1, X)) robot.drivetrain.breakFollowing();
                if (!robot.drivetrain.isBusy()) autoScoring = false;

                robot.drivetrain.update();
                
            } else {

                if (keyPressed(1, X)) robot.startAutoDrive();
                if (gamepadEx2.isDown(LEFT_BUMPER) && gamepadEx2.isDown(RIGHT_BUMPER)) robot.scanner.reset();
                teleOpControls();
            }

            robot.run();

            mTelemetry.addData("Scoring mode", autoScoring ? "auto" : "manual");
            mTelemetry.addLine();
            robot.printTelemetry();
            mTelemetry.update();
        }
        robot.scanner.stop();
    }
}
