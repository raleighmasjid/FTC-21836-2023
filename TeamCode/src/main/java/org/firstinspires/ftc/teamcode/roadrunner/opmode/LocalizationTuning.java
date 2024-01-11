package org.firstinspires.ftc.teamcode.roadrunner.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import static org.firstinspires.ftc.teamcode.roadrunner.ThreeWheelTrackingLocalizer.X_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.roadrunner.ThreeWheelTrackingLocalizer.Y_MULTIPLIER;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drivetrains.MecanumDrivetrain;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
@Config
public class LocalizationTuning extends LinearOpMode {

    public static double
            X_MEASUREMENT = 20,
            Y_MEASUREMENT = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivetrain drive = new MecanumDrivetrain(hardwareMap);

        drive.setMode(RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(FLOAT);

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()) {
            gamepadEx1.readButtons();

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            if (gamepadEx1.wasJustPressed(X)) {
                X_MULTIPLIER *= (X_MEASUREMENT / 2.54) / poseEstimate.getX();
            }

            if (gamepadEx1.wasJustPressed(Y)) {
                Y_MULTIPLIER *= (Y_MEASUREMENT / 2.54) / poseEstimate.getY();
            }
        }
    }
}
