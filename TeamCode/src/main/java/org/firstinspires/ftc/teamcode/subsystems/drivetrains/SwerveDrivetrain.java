package org.firstinspires.ftc.teamcode.subsystems.drivetrains;

import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.LOGO_FACING_DIR;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.USB_FACING_DIR;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot.maxVoltage;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrains.SwerveModule.SwerveModuleID.BL;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrains.SwerveModule.SwerveModuleID.BR;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrains.SwerveModule.SwerveModuleID.FL;
import static org.firstinspires.ftc.teamcode.subsystems.drivetrains.SwerveModule.SwerveModuleID.FR;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;
import static java.util.Arrays.asList;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.filters.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.opmodes.EditablePose;
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.roadrunner.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.subsystems.utilities.sensors.HeadingIMU;

import java.util.ArrayList;
import java.util.List;

@Config
public final class SwerveDrivetrain implements Drivetrain {
    public static PIDCoefficients
            TRANSLATIONAL_PID = new PIDCoefficients(
                8,
                8,
                2
            ),
            HEADING_PID = new PIDCoefficients(
                    8,
                    12,
                    1
            );


    private final SwerveModule[] modules;

    public static EditablePose admissibleError = new EditablePose(0.01, 0.01, 0.001);

    private final TrajectorySequenceRunner trajectorySequenceRunner;

//    private final ThreeWheelTrackingLocalizer localizer;

    private final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    private final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(DriveConstants.MAX_ACCEL);

    private final VoltageSensor batteryVoltageSensor;

    public SwerveDrivetrain(HardwareMap hardwareMap) {

        modules = new SwerveModule[]{
                new SwerveModule(hardwareMap, BR),
                new SwerveModule(hardwareMap, BL),
                new SwerveModule(hardwareMap, FR),
                new SwerveModule(hardwareMap, FL),
        };


        TrajectoryFollower follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                admissibleError.toPose2d(), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // TODO: adjust the names of the following hardware devices to match your configuration

        setZeroPowerBehavior(BRAKE);

        // TODO: if desired, use setLocalizer() to change the localization method
//        localizer = new ThreeWheelTrackingLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>());

        imu = new HeadingIMU(hardwareMap, "imu", new RevHubOrientationOnRobot(LOGO_FACING_DIR, USB_FACING_DIR));
        setCurrentHeading(0);

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                new ArrayList<>(), new ArrayList<>(), new ArrayList<>(), new ArrayList<>()
        );
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        rawHeading = imu.getHeading();

        modules[BR.o].readSensors();
        modules[BL.o].readSensors();
        modules[FR.o].readSensors();
        modules[FL.o].readSensors();

//        DriveSignal signal = trajectorySequenceRunner.update(localizer.getPoseEstimate(), localizer.getPoseVelocity());
//        if (signal != null) setDriveSignal(signal);
    }

    private void setDriveSignal(DriveSignal signal) {
        SwerveModule.State[] velocities = SwerveKinematics.robotToPodStates(signal.getVel());
        SwerveModule.State[] accelerations = SwerveKinematics.robotToPodStates(signal.getAccel());

        List<Double> vels = asList(
                velocities[BR.o].velo,
                velocities[BL.o].velo,
                velocities[FR.o].velo,
                velocities[FL.o].velo
        );

        List<Double> accels = asList(
                accelerations[BR.o].velo,
                accelerations[BL.o].velo,
                accelerations[FR.o].velo,
                accelerations[FL.o].velo
        );

        List<Double> powers = Kinematics.calculateMotorFeedforward(vels, accels, kV, kA, kStatic);

        setModules(
                new SwerveModule.State(powers.get(BR.o), velocities[BR.o].theta),
                new SwerveModule.State(powers.get(BL.o), velocities[BL.o].theta),
                new SwerveModule.State(powers.get(FR.o), velocities[FR.o].theta),
                new SwerveModule.State(powers.get(FL.o), velocities[FL.o].theta)
        );
    }

    private void setModules(SwerveModule.State... states) {

        double vBR = states[BR.o].velo;
        double vBL = states[BL.o].velo;
        double vFR = states[FR.o].velo;
        double vFL = states[FL.o].velo;

        // Get max of all motor power commands
        double max = max(1.0, max(
                max(abs(vBR), abs(vBL)),
                max(abs(vFR), abs(vFL))
        ));

        // Normalize motor powers to [-1, 1]
        vBR /= max;
        vBL /= max;
        vFR /= max;
        vFL /= max;

        modules[BR.o].setVelo(vBR);
        modules[BL.o].setVelo(vBL);
        modules[FR.o].setVelo(vFR);
        modules[FL.o].setVelo(vFL);

        boolean moving =
                        vBR != 0 ||
                        vBL != 0 ||
                        vFR != 0 ||
                        vFL != 0;

        if (moving) {
            modules[BR.o].setTheta(states[BR.o].theta);
            modules[BL.o].setTheta(states[BL.o].theta);
            modules[FR.o].setTheta(states[FR.o].theta);
            modules[FL.o].setTheta(states[FL.o].theta);
        }

        modules[BR.o].run();
        modules[BL.o].run();
        modules[FR.o].run();
        modules[FL.o].run();
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setZeroPowerBehavior(Motor.ZeroPowerBehavior behavior) {
        for (SwerveModule module : modules) module.setZeroPowerBehavior(behavior);
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public void breakFollowing() {
        trajectorySequenceRunner.breakFollowing();
    }

    public double getRawExternalHeading() {
        return 0;
    }

    public Double getExternalHeadingVelocity() {
        return 0.0;
    }

    public final HeadingIMU imu;

    private double headingOffset, rawHeading;
    public static double SLOW_FACTOR = 0.3, ACCEL_LIMIT = 10000;
    private boolean slowModeLocked = false;

    private final SlewRateLimiter
            xRateLimiter = new SlewRateLimiter(ACCEL_LIMIT),
            yRateLimiter = new SlewRateLimiter(ACCEL_LIMIT);

    /**
     * Set internal heading of the robot to correct field-centric direction
     *
     * @param angle Angle of the robot in radians, 0 facing forward and increases counter-clockwise
     */
    public void setCurrentHeading(double angle) {
        headingOffset = normalizeRadians(getRawHeading() - angle);
    }

    public double getHeading() {
        return normalizeRadians(getRawHeading() - headingOffset);
    }

    private double getRawHeading() {
        return rawHeading;
    }

    /**
     * Field-centric driving using {@link HeadingIMU}
     *
     * @param xCommand strafing input
     * @param yCommand forward input
     * @param turnCommand turning input
     */
    public void run(double xCommand, double yCommand, double turnCommand, boolean useSlowMode) {

        xRateLimiter.setLimit(ACCEL_LIMIT);
        yRateLimiter.setLimit(ACCEL_LIMIT);

        xCommand = xRateLimiter.calculate(xCommand);
        yCommand = yRateLimiter.calculate(yCommand);

        // counter-rotate translation vector by current heading
        double
                theta = -getHeading(),
                cos = cos(theta),
                sin = sin(theta),
                x = xCommand,
                y = yCommand;

        xCommand = x * cos - y * sin;
        yCommand = y * cos + x * sin;

        if (useSlowMode) slowModeLocked = false;
        if (useSlowMode || slowModeLocked) {
            yCommand *= SLOW_FACTOR;
            xCommand *= SLOW_FACTOR;
            turnCommand *= SLOW_FACTOR;
        }

        double voltageScalar = maxVoltage / batteryVoltageSensor.getVoltage();

        yCommand *= voltageScalar;
        xCommand *= voltageScalar;
        turnCommand *= voltageScalar;

        // run motors
        setDrivePower(new Pose2d(
                yCommand,
                -xCommand,
                -turnCommand
        ));
    }

    public void setDrivePower(Pose2d drive) {
        setModules(SwerveKinematics.robotToPodStates(drive));
    }

    public void lockSlowMode() {
        this.slowModeLocked = true;
    }

    public void printNumericalTelemetry() {
        mTelemetry.addData("Current heading (radians)", getHeading());
        mTelemetry.addData("Current heading (degrees)", toDegrees(getHeading()));
    }

    @Override
    public void setPoseEstimate(Pose2d pose) {
    }

    @Override
    public Pose2d getPoseEstimate() {
        return null;
    }

    @Config
    public static final class SwerveKinematics {

        public static double
                WIDTH = 10,
                LENGTH = 10;

        public static SwerveModule.State[] robotToPodStates(Pose2d drive) {

            double

            iY = drive.getY(),
            iX = drive.getX(),
            t = drive.getHeading() / hypot(WIDTH, LENGTH),

            // Calculate rotation component vectors
            tY = t * LENGTH,
            tX = t * WIDTH,

            // Combine component vectors into total x and y vectors (per pod)
            a = iY - tY,
            b = iY + tY,
            c = iX - tX,
            d = iX + tX,

            // Get velocity vector (hypotenuse) of total x and y vectors (per pod)
            vBR = hypot(a, d),
            vBL = hypot(a, c),
            vFR = hypot(b, d),
            vFL = hypot(b, c),

            // Calculate pod angles with total x and y vectors (per pod)
            aBR = atan2(a, d),
            aBL = atan2(a, c),
            aFR = atan2(b, d),
            aFL = atan2(b, c);

            return new SwerveModule.State[]{
                new SwerveModule.State(vBR, aBR),
                new SwerveModule.State(vBL, aBL),
                new SwerveModule.State(vFR, aFR),
                new SwerveModule.State(vFL, aFL),
            };
        }
    }
}
