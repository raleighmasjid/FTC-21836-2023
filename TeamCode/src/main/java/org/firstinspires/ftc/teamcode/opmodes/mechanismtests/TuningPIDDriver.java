package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.motion.PIDDriver;
import org.firstinspires.ftc.teamcode.opmodes.EditablePose;
import org.firstinspires.ftc.teamcode.subsystems.drivetrains.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;


@TeleOp(group = "Single mechanism test")
@Config
public final class TuningPIDDriver extends LinearOpMode {

    public static EditablePose target = new EditablePose(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize gamepads:

        BulkReader bulkReader = new BulkReader(hardwareMap);
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Drivetrain drivetrain = new MecanumDrivetrain(hardwareMap);

        PIDDriver driver = new PIDDriver();

        waitForStart();

        while (opModeIsActive()) {
            bulkReader.bulkRead();

            Pose2d current = drivetrain.getPoseEstimate();

            if (gamepad1.x) {
                boolean done = driver.driveTo(drivetrain, target.toPose2d());

                mTelemetry.addLine(done ? "Target reached" : "Moving to target");
                mTelemetry.addLine();
            }
            mTelemetry.addData("Current X", current.getX());
            mTelemetry.addData("Current Y", current.getY());
            mTelemetry.addData("Current heading", normalizeRadians(current.getHeading()));
            mTelemetry.addLine();
            mTelemetry.addData("Target X", target.x);
            mTelemetry.addData("Target Y", target.y);
            mTelemetry.addData("Target heading", normalizeRadians(target.heading));
            mTelemetry.update();
        }
    }
}
