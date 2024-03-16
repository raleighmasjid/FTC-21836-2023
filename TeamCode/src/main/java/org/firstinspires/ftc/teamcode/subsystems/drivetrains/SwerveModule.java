package org.firstinspires.ftc.teamcode.subsystems.drivetrains;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.BARE;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.maxVoltage;

import static java.lang.Math.signum;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.motion.swerve.SwervePodState;

@Config
public final class SwerveModule {

    public static double
            RATIO_TOP_PULLEY_TO_MOTOR = 39.0 / 26.0,
            RATIO_RPM_SERVO_TO_MOTOR = 10.019166666666667 / 6000.0,
            ANTI_COAX_EFFECT_GAIN = 0,
            kS_SERVO = 0,
            OFFSET_BR = 0,
            OFFSET_BL = 0,
            OFFSET_FR = 0,
            OFFSET_FL = 0;

    private final double podRotOffset;
    private final SwervePodState current, target;

    private final MotorEx motor;
    private final CRServo servo;
    private final VoltageSensor batteryVoltageSensor;

    public SwerveModule(HardwareMap hardwareMap, String motorName, String servoName, double podRotOffset) {

        this.motor = new MotorEx(hardwareMap, motorName, BARE);
        this.motor.setZeroPowerBehavior(BRAKE);

        this.servo = new CRServo(hardwareMap, servoName);

        this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        this.podRotOffset = podRotOffset;

        this.target = this.current = new SwervePodState(0, 0);
    }

    public void readSensors() {
        current.theta = normalizeRadians( - podRotOffset);
    }

    public void setVelo(SwervePodState target) {
        this.target.velo = target.velo;
    }

    public void setAngle(SwervePodState target) {
        this.target.theta = target.theta;
    }

    public void run() {

        double scalar = maxVoltage / batteryVoltageSensor.getVoltage();

        target.optimize(current);

        // calculate error
        // calculate target
        // update controller gains
        // set controller
        double pidOutput = 0;
        double staticFF = kS_SERVO * signum(pidOutput) * scalar;

        double motorPower = target.velo * scalar;
        double servoPower = pidOutput + staticFF;

        double wheelCoaxOffset = servoPower * RATIO_RPM_SERVO_TO_MOTOR * RATIO_TOP_PULLEY_TO_MOTOR;
        double podCoaxOffset = motorPower * ANTI_COAX_EFFECT_GAIN;

        motor.set(motorPower + wheelCoaxOffset);
        servo.set(servoPower + podCoaxOffset);
    }

}
