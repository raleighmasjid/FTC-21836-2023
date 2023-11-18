package org.firstinspires.ftc.teamcode.subsystems.drivetrains;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.LowPassGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;

@Config
public class AutoTurnMecanum extends MecanumDrivetrain {

    public static double
            kStatic = 0.0,
            TURN_SETTLING_TIME = 0.0,
            TRANSLATION_SETTLING_TIME = 0.0;

    public static LowPassGains derivFilterGains = new LowPassGains(
            0.5,
            10
    );

    public static PIDGains pidGains = new PIDGains(
            0.0,
            0.0,
            0.0,
            0.0
    );

    private boolean useAutoTurn = true;

    private double lastXCommand, lastYCommand, targetHeading;

    private final ElapsedTime turnSettlingTimer = new ElapsedTime();
    private final ElapsedTime translationSettlingTimer = new ElapsedTime();

    private final FIRLowPassFilter kDFilter = new FIRLowPassFilter(derivFilterGains);
    private final PIDController headingController = new PIDController(kDFilter);

    public AutoTurnMecanum(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void updateGains() {
        headingController.setGains(pidGains);
        kDFilter.setGains(derivFilterGains);
    }

    @Override
    public void run(double xCommand, double yCommand, double turnCommand) {
        double scalar = 12.0 / batteryVoltageSensor.getVoltage();
        boolean useManualInput = turnCommand != 0.0;

        if (useManualInput || !useAutoTurn) {
            turnCommand *= scalar;
            turnSettlingTimer.reset();
        }

        boolean xStopped = lastXCommand != 0 && xCommand == 0;
        boolean yStopped = lastYCommand != 0 && yCommand == 0;
        if (xStopped || yStopped) translationSettlingTimer.reset();

        if (useAutoTurn) {
            if (useManualInput || turnSettlingTimer.seconds() <= TURN_SETTLING_TIME) {
                setTargetHeading(getHeading());
            } else if (translationSettlingTimer.seconds() > TRANSLATION_SETTLING_TIME) {
                headingController.setError(-normalizeRadians(targetHeading - getHeading()));
                double pidOutput = headingController.calculate(new State(getHeading()));
                turnCommand = pidOutput + (Math.signum(pidOutput) * kStatic * scalar);
            }
        }

        lastXCommand = xCommand;
        lastYCommand = yCommand;
        super.run(xCommand * scalar, yCommand * scalar, turnCommand);
    }


    /**
     * Set target heading of the robot to turn to automatically (and lock to)
     *
     * @param angle Angle of the robot in radians, 0 facing forward and increases counter-clockwise
     */
    public void setTargetHeading(double angle) {
        targetHeading = normalizeRadians(angle);
    }

    @Override
    public void setCurrentHeading(double angle) {
        super.setCurrentHeading(angle);
        setTargetHeading(angle);
    }

    public void toggleAutoTurn() {
        useAutoTurn = !useAutoTurn;
    }

    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Auto turn is", useAutoTurn ? "active" : "inactive");
    }

    @Override
    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        super.printNumericalTelemetry(telemetry);
        telemetry.addLine();
        telemetry.addData("Target heading (radians)", targetHeading);
        telemetry.addData("Target heading (degrees)", toDegrees(targetHeading));
        telemetry.addLine();
        telemetry.addData("Heading error derivative (ticks/s)", headingController.getErrorDerivative());
    }
}
