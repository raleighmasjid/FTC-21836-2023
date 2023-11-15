package org.firstinspires.ftc.teamcode.subsystems.drivetrains;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.Collections;


public class MecanumDrivetrain {

    public final ThreadedIMU imu;

    private final MecanumDrive mecanumDrivetrain;

    protected final MotorEx[] motors;

    private double headingOffset, latestIMUReading;

    private final HardwareMap hardwareMap;

    protected final VoltageSensor batteryVoltageSensor;

    protected MotorEx getMotor(String name) {
        return new MotorEx(hardwareMap, name, 537.7, 312);
    }

    public MecanumDrivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Assign motors using their hardware map names, each drive-type can have different names if needed
        motors = new MotorEx[]{
                getMotor("left front"),
                getMotor("right front"),
                getMotor("left back"),
                getMotor("right back")
        };

        motors[0].setInverted(true);
        motors[1].setInverted(false);
        motors[2].setInverted(true);
        motors[3].setInverted(false);

        for (MotorEx motor : motors) motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Initialize the FTCLib drive-base
        mecanumDrivetrain = new MecanumDrive(false, motors[0], motors[1], motors[2], motors[3]);

        this.imu = new ThreadedIMU(hardwareMap, "imu", new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
    }

    public void readIMU() {
        latestIMUReading = imu.getHeading();
    }

    /**
     * Set internal heading of the robot to correct field-centric direction
     *
     * @param angle Angle of the robot in degrees, 0 facing forward and increases counter-clockwise
     */
    public void setCurrentHeading(double angle) {
        headingOffset = latestIMUReading - AngleUnit.normalizeDegrees(angle);
    }

    public double getHeading() {
        return AngleUnit.normalizeDegrees(latestIMUReading - headingOffset);
    }

    public void run(double xCommand, double yCommand, double turnCommand) {
        // normalize inputs
        double max = Collections.max(Arrays.asList(Math.abs(xCommand), Math.abs(yCommand), Math.abs(turnCommand), 1.0));
        xCommand /= max;
        yCommand /= max;
        turnCommand /= max;

        mecanumDrivetrain.driveFieldCentric(xCommand, yCommand, turnCommand, getHeading());
    }

    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Drivetrain current heading", getHeading());
    }
}

