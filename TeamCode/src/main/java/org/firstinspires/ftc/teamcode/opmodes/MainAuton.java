package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_SECOND;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRed;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRight;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.AutonPixelSupplier.Randomization.CENTER;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.AutonPixelSupplier.Randomization.randomizations;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.AutonPixelSupplier;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel;

import java.util.ArrayList;

@Config
@Autonomous(preselectTeleOp = "AutomatedTeleOp")
public final class MainAuton extends LinearOpMode {

    public static final double
            LEFT = PI,
            FORWARD = 1.5707963267948966,
            RIGHT = 0,
            BACKWARD = -1.5707963267948966;

    // Declare objects:
    public static GamepadEx gamepadEx1, gamepadEx2;
    public static MultipleTelemetry mTelemetry;
    static Robot robot;
    public static Backdrop autonBackdrop = new Backdrop();
    static Pose2d autonEndPose = new Pose2d(0, 0, FORWARD);

    public static boolean keyPressed(int gamepad, GamepadKeys.Button button) {
        return (gamepad == 2 ? gamepadEx2 : gamepadEx1).wasJustPressed(button);
    }

    public static double
            X_START_LEFT = -35,
            X_START_RIGHT = 12,
            X_AFTER_SPIKE = 24;

    public static EditablePose
            startPose = new EditablePose(X_START_RIGHT, -61.788975, FORWARD),
            centerSpike = new EditablePose(X_START_RIGHT, -30, FORWARD),
            leftSpike = new EditablePose(7, -40, toRadians(120)),
            rightSpike = new EditablePose(24 - leftSpike.x, leftSpike.y, LEFT - leftSpike.heading);

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry);

        // Initialize robot:
        robot = new Robot(hardwareMap);
        robot.preload();

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        boolean partnerWillDoRand = false;
        // Get gamepad 1 button input and save alliance and side for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (keyPressed(1, DPAD_RIGHT))  isRight = true;
            if (keyPressed(1, DPAD_LEFT))   isRight = false;
            if (keyPressed(1, B))           isRed = true;
            if (keyPressed(1, X))           isRed = false;
            if (keyPressed(1, Y))           partnerWillDoRand = !partnerWillDoRand;
            mTelemetry.addLine("Selected " + (isRed ? "RED" : "BLUE") + " " + (isRight ? "RIGHT" : "LEFT"));
            mTelemetry.addLine("Your alliance partner will " + (partnerWillDoRand ? "not " : "") + "be placing a yellow pixel");
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }
        mTelemetry.addLine("Confirmed " + (isRed ? "RED" : "BLUE") + " " + (isRight ? "RIGHT" : "LEFT"));
        mTelemetry.update();

        TrajectorySequence[] sequences = new TrajectorySequence[3];

        Pose2d startPose = MainAuton.startPose.byBoth().toPose2d();
        robot.drivetrain.setPoseEstimate(startPose);

        for (AutonPixelSupplier.Randomization rand : randomizations) {
            ArrayList<Pixel> placements = AutonPixelSupplier.getPlacements(rand, partnerWillDoRand);

            AutonPixelSupplier.Randomization tRand = rand;
            if (!isRed) {
                if (rand == AutonPixelSupplier.Randomization.RIGHT) tRand = AutonPixelSupplier.Randomization.LEFT;
                else if (rand == AutonPixelSupplier.Randomization.LEFT) tRand = AutonPixelSupplier.Randomization.RIGHT;
            }
            if (partnerWillDoRand) placements.remove(0);

            Pose2d spike = (
                    tRand == AutonPixelSupplier.Randomization.LEFT ? leftSpike :
                            tRand == AutonPixelSupplier.Randomization.RIGHT ? rightSpike :
                                    centerSpike).byBoth().toPose2d();

            Pose2d afterSpike = new Pose2d(spike.getX() + X_AFTER_SPIKE, spike.getY(), LEFT);

            sequences[rand.ordinal()] = robot.drivetrain.trajectorySequenceBuilder(startPose)
                    .setTangent(FORWARD)
                    .splineTo(spike.vec(), spike.getHeading())
                    .setTangent(RIGHT)
                    .splineToLinearHeading(afterSpike, RIGHT)
                    .addTemporalMarker(() -> {
                        robot.deposit.lift.setTargetRow(placements.get(0).y);
                        robot.intake.setRequiredIntakingAmount(2);
                    })
                    .lineToSplineHeading(placements.get(0).toPose2d())
                    .addTemporalMarker(() -> {
                        robot.deposit.paintbrush.dropPixels(2);
                        autonBackdrop.add(placements.get(0));
                    })
                    .waitSeconds(TIME_DROP_SECOND)
                    .build()
            ;
        }

        robot.preload();
        waitForStart();
        robot.drivetrain.followTrajectorySequenceAsync(sequences[CENTER.ordinal()]);

        // Control loop:
        while (opModeIsActive()) {
            // Manually clear old sensor data from the last loop:
            robot.readSensors();

            robot.drivetrain.update();
            robot.run();

            autonEndPose = robot.drivetrain.getPoseEstimate();
            // Push telemetry data
            robot.printTelemetry();
            mTelemetry.update();
        }
    }

    private static class EditablePose {

        public double x, y, heading;

        private EditablePose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        private EditablePose byAlliance() {
            double alliance = isRed ? 1 : -1;
            y *= alliance;
            heading *= alliance;
            return this;
        }

        private EditablePose bySide() {
            x += isRight == isRed ? 0 : X_START_LEFT - X_START_RIGHT;
            return this;
        }

        private EditablePose byBoth() {
            return byAlliance().bySide();
        }

        private Pose2d toPose2d() {
            return new Pose2d(x, y, heading);
        }
    }
}
