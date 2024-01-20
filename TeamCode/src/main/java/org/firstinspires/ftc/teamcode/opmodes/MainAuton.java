package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline.Randomization.randomizations;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.EditablePose.backdropSide;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Deposit.Paintbrush.TIME_DROP_SECOND;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Intake.Height.FIVE_STACK;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.isRed;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;
import static java.util.Collections.swap;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.vision.PropDetectPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.AutonPixelSupplier;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel;
import org.firstinspires.ftc.teamcode.subsystems.utilities.sensors.TeamPropDetector;

import java.util.ArrayList;

@Config
@Autonomous(preselectTeleOp = "MainTeleOp")
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
            X_SHIFT_AFTER_SPIKE = 5,
            FORWARD_BEFORE_SPIKE = 17,
            X_TILE = 24,
            CYCLES_BACKDROP_SIDE = 0,
            CYCLES_AUDIENCE_SIDE = 0;

    public static EditablePose
            startPose = new EditablePose(X_START_RIGHT, -61.788975, FORWARD),
            centerSpike = new EditablePose(X_START_RIGHT, -26, FORWARD),
            leftSpike = new EditablePose(2.5, -36, toRadians(150)),
            toParkInner = new EditablePose(Backdrop.X, -60, LEFT),
            parkedInner = new EditablePose(60, -60, LEFT);

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

        boolean partnerWillDoRand = false, doCycles = true;
        // Get gamepad 1 button input and save alliance and side for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (keyPressed(1, B))           isRed = true;
            if (keyPressed(1, X))           isRed = false;
            if (keyPressed(1, DPAD_RIGHT))  backdropSide = isRed;
            if (keyPressed(1, DPAD_LEFT))   backdropSide = !isRed;
            if (keyPressed(1, DPAD_UP))     doCycles = true;
            if (keyPressed(1, DPAD_DOWN))   doCycles = false;
            if (keyPressed(1, Y))           partnerWillDoRand = !partnerWillDoRand;
            mTelemetry.addLine("Selected " + (isRed ? "RED " : "BLUE ") + (backdropSide ? "BACKDROP " : "AUDIENCE ") + "side");
            mTelemetry.addLine();
            mTelemetry.addLine("Your alliance PARTNER WILL " + (partnerWillDoRand ? "" : "NOT ") + "PLACE YELLOW");
            mTelemetry.addLine();
            mTelemetry.addLine("You WILL " + (doCycles ? "CYCLE" : "PARK"));
            mTelemetry.addLine();
            mTelemetry.addLine("Press both shoulder buttons to CONFIRM!");
            mTelemetry.update();
        }
        mTelemetry.addLine("Confirmed " + (isRed ? "RED" : "BLUE") + " " + (backdropSide ? "BACKDROP" : "AUDIENCE") + " side");
        mTelemetry.update();

        Pose2d startPose = MainAuton.startPose.byBoth().toPose2d();
        robot.drivetrain.setPoseEstimate(startPose);

        PropDetectPipeline.Randomization location = PropDetectPipeline.Randomization.RIGHT;
        TeamPropDetector detector = new TeamPropDetector(hardwareMap);

        TrajectorySequence[] sequences = new TrajectorySequence[3];
        EditablePose rightSpike = new EditablePose(X_TILE - leftSpike.x, leftSpike.y, LEFT - leftSpike.heading);

        for (PropDetectPipeline.Randomization rand : randomizations) {

            ArrayList<Pixel> placements = AutonPixelSupplier.getPlacements(rand, partnerWillDoRand);
            if (partnerWillDoRand) placements.remove(0);
            if (!backdropSide) swap(placements, 0, 1);

            Pose2d spike;
            switch (rand) {
                case LEFT:
                    spike = (isRed ? leftSpike : rightSpike).byBoth().toPose2d();
                    break;
                case RIGHT:
                    spike = (isRed ? rightSpike : leftSpike).byBoth().toPose2d();
                    break;
                case CENTER:
                default:
                    spike = centerSpike.byBoth().toPose2d();
            }

            Pose2d afterSpike = new EditablePose(spike.getX() + X_SHIFT_AFTER_SPIKE, spike.getY(), LEFT).flipBySide().byAlliance().toPose2d();

            TrajectorySequenceBuilder sequence = robot.drivetrain.trajectorySequenceBuilder(startPose)
                    .setTangent(startPose.getHeading())
                    .forward(FORWARD_BEFORE_SPIKE)
                    .splineTo(spike.vec(), spike.getHeading())
                    .setTangent(afterSpike.getHeading())
                    .lineToSplineHeading(afterSpike)
            ;

            if (backdropSide) {

                sequence
                        .addTemporalMarker(() -> {
                            robot.deposit.lift.setTargetRow(placements.get(0).y);
                            robot.intake.setRequiredIntakingAmount(2);
                        })
                        .splineToSplineHeading(placements.get(0).toPose2d(), RIGHT)
                        .addTemporalMarker(() -> {
                            robot.deposit.paintbrush.dropPixels(2);
                            autonBackdrop.add(placements.get(0));
                        })
                        .waitSeconds(TIME_DROP_SECOND)
                ;

            } else {

                sequence
                        .addTemporalMarker(() -> {
                            robot.intake.setRequiredIntakingAmount(1);
                            robot.intake.setHeight(FIVE_STACK);
                        })
                ;

            }

            if (!doCycles) {
                sequence
                        .lineTo(toParkInner.byAlliance().toPose2d().vec())
                        .lineTo(parkedInner.byAlliance().toPose2d().vec())
                ;
            } else for (int i = 1; i <= (backdropSide ? CYCLES_BACKDROP_SIDE : CYCLES_AUDIENCE_SIDE); i++) {
                // TODO Add cycling pathing
            }

            sequences[rand.ordinal()] = sequence.build();
        }

        while (opModeInInit()) {
            mTelemetry.addData("Location", (location = detector.run()).name());
            mTelemetry.update();
        }

        detector.stop();
        robot.drivetrain.followTrajectorySequenceAsync(sequences[location.ordinal()]);

        // Control loop:
        while (opModeIsActive()) {
            // Manually clear old sensor data from the last loop:
            robot.readSensors();
            robot.run();

            autonEndPose = robot.drivetrain.getPoseEstimate();
            // Push telemetry data
            robot.printTelemetry();
            mTelemetry.update();
        }
    }

    public static class EditablePose {

        public double x, y, heading;

        static boolean backdropSide = true;

        public EditablePose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        public EditablePose byAlliance() {
            double alliance = isRed ? 1 : -1;
            y *= alliance;
            heading *= alliance;
            return this;
        }

        public EditablePose bySide() {
            if (!backdropSide) x += X_START_LEFT - X_START_RIGHT;
            return this;
        }

        public EditablePose byBoth() {
            return byAlliance().bySide();
        }

        public Pose2d toPose2d() {
            return new Pose2d(x, y, heading);
        }

        public EditablePose flipBySide() {
            if (!backdropSide) heading = Math.PI - heading;
            if (!backdropSide) x = (X_START_LEFT + X_START_RIGHT) - x;
            return this;
        }
    }
}
