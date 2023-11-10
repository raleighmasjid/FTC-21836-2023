package org.firstinspires.ftc.teamcode.subsystems.drivetrains;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HeadingIMU implements HeadingLocalizer {

    private final IMU imu;

    public HeadingIMU(HardwareMap hw, String name, RevHubOrientationOnRobot imuOrientation) {
        imu = hw.get(IMU.class, name);
        imu.resetDeviceConfigurationForOpMode();
        imu.resetYaw();
        imu.initialize(new IMU.Parameters(imuOrientation));
    }

    @Override
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
