package org.firstinspires.ftc.teamcode.subsystems.drivetrains;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.control.Differentiator;

public class HeadingIMU extends Thread implements HeadingLocalizer {

    private final IMU imu;

    private double heading;

    private boolean run = false;

    private final Differentiator veloCalc;
    private double angularVelo;

    public HeadingIMU(HardwareMap hw, String name, RevHubOrientationOnRobot imuOrientation) {
        imu = hw.get(IMU.class, name);
        imu.resetDeviceConfigurationForOpMode();
        imu.resetYaw();
        imu.initialize(new IMU.Parameters(imuOrientation));

        veloCalc = new Differentiator();
    }

    @Override
    public void start() {
        run = true;
        super.start();
    }

    @Override
    public void run() {
        while (run) {
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            angularVelo = veloCalc.getDerivative(heading);
        }
    }

    @Override
    public double getHeading() {
        return heading;
    }

    public double getAngularVelo() {
        return angularVelo;
    }

    @Override
    public void interrupt() {
        run = false;
        super.interrupt();
    }
}
