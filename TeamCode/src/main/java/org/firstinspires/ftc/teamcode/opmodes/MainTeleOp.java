package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_STICK_BUTTON;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.WHITE;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.BACKWARD;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.EditablePose.backdropSide;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.FORWARD;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.autonEndPose;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx2;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.loopMod;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.TeleOpConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.TeleOpConfig.EDITING_AUTO_SLOW;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.TeleOpConfig.EDITING_SIDE;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.TeleOpConfig.EDITING_SLOW_LOCK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Lift.HEIGHT_CLIMBING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FIVE_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FOUR_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.THREE_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.TWO_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRed;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;

@TeleOp
public final class MainTeleOp extends LinearOpMode {

    static boolean autoSlowMode = true;

    @Override
    public void runOpMode() throws InterruptedException {

        teleOpInit(this);

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            teleOpControls();

            robot.run();

            robot.printTelemetry();
            mTelemetry.update();
        }
    }

    enum TeleOpConfig {
        EDITING_ALLIANCE,
        EDITING_SIDE,
        EDITING_SLOW_LOCK,
        EDITING_AUTO_SLOW;

        public static final TeleOpConfig[] selections = values();

        public TeleOpConfig plus(int i) {
            return selections[loopMod(ordinal() + i, selections.length)];
        }
        public String markIf(TeleOpConfig s) {
            return this == s ? " <" : "";
        }
    }

    static void teleOpInit(LinearOpMode opMode) {
        boolean isAutomated = opMode instanceof AutomatedTeleOp;

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize robot:
        robot = new Robot(opMode.hardwareMap);
        robot.initRun();
        if (isAutomated) robot.startAlgorithm(opMode.hardwareMap);

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(opMode.gamepad1);
        gamepadEx2 = new GamepadEx(opMode.gamepad2);

        TeleOpConfig selection = EDITING_ALLIANCE;

        boolean lockSlowMode = false;
        // Get gamepad 1 button input and locks slow mode:
        while (opMode.opModeInInit()) {
            gamepadEx1.readButtons();

            if (keyPressed(1, DPAD_UP))   selection = selection.plus(-1);
            if (keyPressed(1, DPAD_DOWN)) selection = selection.plus(1);

            if (keyPressed(1, X)) switch (selection) {
                case EDITING_ALLIANCE:
                    isRed = !isRed;
                    break;
                case EDITING_SIDE:
                    backdropSide = !backdropSide;
                    break;
                case EDITING_AUTO_SLOW:
                    autoSlowMode = !autoSlowMode;
                    break;
                default:
                case EDITING_SLOW_LOCK:
                    lockSlowMode = !lockSlowMode;
                    break;
            }

            mTelemetry.addLine((isRed ? "RED " : "BLUE ") + selection.markIf(EDITING_ALLIANCE));
            mTelemetry.addLine();
            mTelemetry.addLine((backdropSide ? "BACKDROP " : "AUDIENCE ") + "side" + selection.markIf(EDITING_SIDE));
            mTelemetry.addLine();
            mTelemetry.addLine("Auto slow is" + (autoSlowMode ? "enabled" : "disabled") + selection.markIf(EDITING_AUTO_SLOW));
            mTelemetry.addLine();
            mTelemetry.addLine((lockSlowMode ? "SLOW" : "NORMAL") + selection.markIf(EDITING_SLOW_LOCK));

            mTelemetry.update();
        }

        if (lockSlowMode) robot.drivetrain.lockSlowMode();

        if (isAutomated) robot.autoScoringManager.backdropScanner.pipeline.isRed = isRed;

        if (autonEndPose == null) autonEndPose = MainAuton.startPose.byBoth().toPose2d();
        robot.drivetrain.setPoseEstimate(autonEndPose);
        robot.drivetrain.setCurrentHeading(autonEndPose.getHeading() - (isRed ? FORWARD : BACKWARD));
    }

    static void teleOpControls() {
        robot.intake.setMotorPower(
                gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER)
        );

        if (gamepadEx2.isDown(LEFT_BUMPER)) {
            if (keyPressed(2, Y))               robot.intake.setDesiredPixelCount(2);
            if (keyPressed(2, X))               robot.intake.setDesiredPixelCount(1);
            if (keyPressed(2, A))               robot.intake.toggle();
            if (keyPressed(2, B))               robot.deposit.lift.setTargetRow(HEIGHT_CLIMBING);

            if (keyPressed(2, RIGHT_STICK_BUTTON))  autoSlowMode = !autoSlowMode;

        } else {

            robot.deposit.lift.setLiftPower(gamepadEx2.getLeftY());
            if (keyPressed(2, LEFT_STICK_BUTTON))   robot.deposit.lift.reset();
            if (keyPressed(2, RIGHT_STICK_BUTTON))  robot.drone.toggle();

            if (keyPressed(2, DPAD_DOWN))       robot.deposit.lift.changeRow(-1);
            else if (keyPressed(2, DPAD_UP))    robot.deposit.lift.changeRow(1);
            else if (keyPressed(2, DPAD_LEFT))  robot.deposit.paintbrush.dropPixel();
            else if (keyPressed(2, DPAD_RIGHT)) robot.deposit.paintbrush.lockPixels(WHITE);

            if (keyPressed(2, Y))               robot.intake.setHeight(FIVE_STACK);
            if (keyPressed(2, X))               robot.intake.setHeight(FOUR_STACK);
            if (keyPressed(2, B))               robot.intake.setHeight(THREE_STACK);
            if (keyPressed(2, A))               robot.intake.setHeight(TWO_STACK);
            if (keyPressed(2, RIGHT_BUMPER))    robot.intake.setHeight(FLOOR);
        }

        if (keyPressed(1, Y))                   robot.spike.toggle();

        double x = gamepadEx1.getRightX();
        if (gamepadEx1.isDown(LEFT_BUMPER)) {
            double y = gamepadEx1.getRightY();
            if (hypot(x, y) >= 0.8) robot.drivetrain.setCurrentHeading(-atan2(y, x) - FORWARD);
            x = 0;
        }

        // Field-centric driving with control stick inputs:
        boolean driveSlow =
                gamepadEx1.isDown(RIGHT_BUMPER) ||
                (autoSlowMode && (
                        (robot.deposit.lift.isScoring() && !robot.deposit.lift.isRetracted()) ||
                        gamepadEx1.getTrigger(RIGHT_TRIGGER) > 0
                ));

        robot.drivetrain.run(
                gamepadEx1.getLeftX(),
                gamepadEx1.getLeftY(),
                x,
                driveSlow
        );
    }
}
