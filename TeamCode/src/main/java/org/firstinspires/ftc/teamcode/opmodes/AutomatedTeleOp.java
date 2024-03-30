package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.opmodes.AutomatedTeleOp.OpState.MANUAL;
import static org.firstinspires.ftc.teamcode.opmodes.AutomatedTeleOp.OpState.MOVING_TO_1;
import static org.firstinspires.ftc.teamcode.opmodes.AutomatedTeleOp.OpState.MOVING_TO_2;
import static org.firstinspires.ftc.teamcode.opmodes.AutomatedTeleOp.OpState.SCORING_1;
import static org.firstinspires.ftc.teamcode.opmodes.AutomatedTeleOp.OpState.SCORING_2;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.autonBackdrop;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx2;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.loopMod;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.toPose2d;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.doAutoSlow;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.teleOpControls;
import static org.firstinspires.ftc.teamcode.opmodes.MainTeleOp.teleOpInit;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_FIRST;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_SECOND;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.motion.PIDDriver;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.control.vision.pipelines.placementalg.Pixel;

@TeleOp
@Config
public final class AutomatedTeleOp extends LinearOpMode {

    enum OpState {
        MANUAL,
        MOVING_TO_1,
        SCORING_1,
        MOVING_TO_2,
        SCORING_2;

        public static final OpState[] states = values();

        public OpState plus(int i) {
            return states[loopMod(ordinal() + i, states.length)];
        }
    }

    private OpState opState = MANUAL;

    private final PIDDriver driver = new PIDDriver();
    private final ElapsedTime timer = new ElapsedTime();

    private final Backdrop backdrop = autonBackdrop;
    private final Pixel[] placements = {new Pixel(0, 0, EMPTY), new Pixel(0, 0, EMPTY)};
    private final Pixel.Color[] wingColors = {EMPTY, EMPTY};

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime loopTimer = new ElapsedTime();

        teleOpInit(this);

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            if (
                    gamepadEx1.getLeftX() != 0 ||
                    gamepadEx1.getLeftY() != 0 ||
                    gamepadEx1.getRightY() != 0
            ) {
                opState = MANUAL;
            } else if (keyPressed(1, X)) {

                if (robot.deposit.paintbrush.pixelsLocked == 2) opState = MOVING_TO_1;
                else if (robot.deposit.paintbrush.pixelsLocked == 1) opState = MOVING_TO_2;

            }

            autoScoringStateMachine();

            robot.run();

            mTelemetry.addData("Loop time", loopTimer.seconds());
            loopTimer.reset();
            mTelemetry.addLine();
            mTelemetry.addLine();

            mTelemetry.addData("Auto slow is", doAutoSlow ? "enabled" : "disabled");
            mTelemetry.addLine();
            robot.printTelemetry();
            mTelemetry.update();
        }
    }

    private void autoScoringStateMachine() {
        switch (opState) {

            default:
            case MANUAL:
                teleOpControls();
                break;

            case MOVING_TO_1:

                robot.deposit.lift.setTargetRow(placements[0].y);

                boolean reached1 = driver.driveTo(robot.drivetrain, toPose2d(placements[0]));

                if (!reached1) break;

                robot.deposit.paintbrush.dropPixel();
                backdrop.add(placements[0]);
                timer.reset();
                opState = SCORING_1;

            case SCORING_1:

                if (timer.seconds() <= TIME_DROP_FIRST) break;

                opState = MOVING_TO_2;

            case MOVING_TO_2:

                robot.deposit.lift.setTargetRow(placements[1].y);

                boolean reached2 = driver.driveTo(robot.drivetrain, toPose2d(placements[1]));

                if (!reached2) break;

                robot.deposit.paintbrush.dropPixel();
                backdrop.add(placements[1]);
                timer.reset();
                opState = SCORING_2;

            case SCORING_2:

                if (timer.seconds() <= TIME_DROP_SECOND) break;

                opState = MANUAL;
        }
    }
}
