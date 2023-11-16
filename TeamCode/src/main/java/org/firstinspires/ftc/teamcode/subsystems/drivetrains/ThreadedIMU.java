package org.firstinspires.ftc.teamcode.subsystems.drivetrains;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ThreadedIMU extends Thread {

    private final IMU imu;

    private double heading, angularVelo;

    private boolean run = true;

    public ThreadedIMU(HardwareMap hw, String name, RevHubOrientationOnRobot imuOrientation) {
        imu = hw.get(IMU.class, name);
        imu.resetDeviceConfigurationForOpMode();
        imu.resetYaw();
        imu.initialize(new IMU.Parameters(imuOrientation));
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void run() {
        while (run) {
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            angularVelo = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        }
    }

    public double getHeading() {
        return heading;
    }

    public double getAngularVelo() {
        return angularVelo;
    }

    @Override
    public void interrupt() {
        run = false;
    }
}
