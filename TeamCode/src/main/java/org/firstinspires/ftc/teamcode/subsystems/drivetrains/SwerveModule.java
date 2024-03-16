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

import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.control.motion.swerve.SwervePodState;

@Config
public final class SwerveModule {

    public static double
            TEETH_MOTOR_PULLEY = 26,
            TEETH_TOP_PULLEY = 39,
            MAX_RPM_SERVO = 10.019166666666667,
            MAX_RPM_MOTOR = 5900,
            MAX_SERVO_TORQUE = 347,
            COAX_EFFECT_TORQUE = 30.675,
            RATIO_RPM_SERVO_TO_MOTOR = MAX_RPM_SERVO / MAX_RPM_MOTOR,
            RATIO_TOP_PULLEY_TO_MOTOR = TEETH_TOP_PULLEY / TEETH_MOTOR_PULLEY,
            COAX_EFFECT_TORQUE_CORRECTION_GAIN = -COAX_EFFECT_TORQUE / MAX_SERVO_TORQUE,
            COAX_EFFECT_WHEEL_CORRECTION_GAIN = RATIO_TOP_PULLEY_TO_MOTOR * RATIO_RPM_SERVO_TO_MOTOR,
            kS_SERVO = 0,
            OFFSET_BR = 0,
            OFFSET_BL = 0,
            OFFSET_FR = 0,
            OFFSET_FL = 0;

    public static PIDGains thetaGains = new PIDGains(
            0,
            0,
            0,
            1
    );

    private final double podRotOffset;
    private final SwervePodState current, target;

    private final MotorEx motor;
    private final CRServo servo;
    private final VoltageSensor batteryVoltageSensor;

    private final PIDController thetaController = new PIDController();

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
        thetaController.setGains(thetaGains);
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

        double thetaTarget = normalizeRadians(target.theta - current.theta) + current.theta;

        thetaController.setTarget(new State(thetaTarget));

        double pidOutput = thetaController.calculate(new State(current.theta));
        
        double staticFF = kS_SERVO * signum(pidOutput) * scalar;

        double motorPower = target.velo * scalar;
        double servoPower = pidOutput + staticFF;

        double coaxWheelCorrection = servoPower * COAX_EFFECT_WHEEL_CORRECTION_GAIN;
        double coaxTorqueCorrection = motorPower * COAX_EFFECT_TORQUE_CORRECTION_GAIN;

        motor.set(motorPower + coaxWheelCorrection);
        servo.set(servoPower + coaxTorqueCorrection);
    }

}
