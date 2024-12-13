package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ModeSet;
import frc.robot.ModeSet.Mode;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase implements Vision.VisionConsumer {
    // A lock to ensure thread-safety when updating odometry.
    static final Lock odometryLock = new ReentrantLock();

    // Interface for the gyro input/output.
    private final GyroIO gyroIO;
    // Holds gyro sensor inputs automatically logged for diagnostics.
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    // Array of swerve modules: front-left, front-right, back-left, back-right.
    private final Module[] modules = new Module[4];

    // Routine for system identification (characterization tests).
    private final SysIdRoutine sysId;

    // Alert shown if gyro is disconnected and fallback mode is used.
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    // Kinematics class converts between chassis speeds and individual module states.
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

    // Current (raw) rotation measured by the gyro.
    private Rotation2d rawGyroRotation = new Rotation2d();

    // Keep track of previous module positions for odometry updates.
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    // Pose estimator that fuses sensor data (gyro, wheel encoders, vision) to estimate robot pose.
    private SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    /**
     * Constructor for the Drive subsystem.
     *
     * @param gyroIO Interface to the gyro hardware.
     * @param flModuleIO Front-left module I/O.
     * @param frModuleIO Front-right module I/O.
     * @param blModuleIO Back-left module I/O.
     * @param brModuleIO Back-right module I/O.
     */
    public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        // Create each module (front-left = 0, front-right = 1, back-left = 2, back-right = 3).
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        // Report the usage to the Hardware Abstraction Layer (for statistical or diagnostic purposes.)
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start the odometry thread (presumably handles asynchronous odometry updates).
        SparkOdometryThread.getInstance().start();

        // Configure the auto builder for path following using PathPlanner.
        AutoBuilder.configure(
                this::getPose, // Function to get current robot pose
                this::resetOdometry, // Function to reset odometry
                this::getChassisSpeeds, // Function to get current chassis speeds
                this::runVelocity, // Function to set desired velocity
                new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                ppConfig, // PathPlanner config (defined in DriveConstants or elsewhere)
                () -> DriverStation.getAlliance().orElse(Alliance.Blue)
                        == Alliance.Red, // Flip paths if on Red Alliance
                this // The subsystem this auto builder will act upon
                );

        // Set the pathfinding algorithm to a local AD* algorithm.(Necessary when using advantage scope sim)
        Pathfinding.setPathfinder(new LocalADStarAK());

        // Set callbacks for PathPlanner logging, so we can log the currently active path.
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        // Set a callback for when target pose updates occur, to log trajectory setpoints.
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

        // Configure SysId routine for characterizing the drivetrain. We pass lambdas that run characterization.
        // CHECK https://docs.wpilib.org/tr/stable/docs/software/advanced-controls/system-identification/index.html

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Possibly unused for now
                        null,
                        null,
                        (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runCharacterization(voltage.in(Volts)), // Apply given voltage to drive
                        null, // Possibly unused
                        this // Subsystem for context
                        ));
    }

    @Override
    public void periodic() {
        // Lock to ensure odometry updates do not conflict with reading data.
        odometryLock.lock();
        // Update gyro sensor inputs.
        gyroIO.updateInputs(gyroInputs);
        // Log gyro inputs for diagnostics.
        Logger.processInputs("Drive/Gyro", gyroInputs);
        // Update each module’s sensor readings and outputs.
        for (var module : modules) {
            module.periodic();
        }
        // Release the lock so other processes can use odometry data.
        odometryLock.unlock();

        // If the robot is disabled, stop the modules to prevent unintended motion.
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // If disabled, log empty setpoints for clarity.
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry using sampled data from modules.
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All modules sample together
        int sampleCount = sampleTimestamps.length;

        for (int i = 0; i < sampleCount; i++) {
            // Get wheel positions at this sample index.
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                // Delta is current position minus the last known position for that module.
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                // Update last known positions to current.
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle.
            if (gyroInputs.connected) {
                // Use real gyro if connected.
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // If gyro is disconnected, estimate rotation from wheel motions.(fallback)
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Update the pose estimator with the new sensor data (timestamp, rotation, and module positions).
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // If gyro is disconnected (and not in sim mode), show an alert.
        gyroDisconnectedAlert.set(!gyroInputs.connected && ModeSet.currentMode != Mode.SIM);
    }

    /**
     * Runs the drive at the desired chassis speeds.
     *
     * @param speeds The desired linear and angular speeds (m/s and rad/s).
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Discretize speeds (possibly to reduce noise).
        speeds.discretize(0.02);

        // Calculate the swerve module states from the chassis speeds.
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);

        // Limit wheel speeds to the maximum allowed speed.
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

        // Log the raw setpoints before optimization (turning wheels to shortest path).
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

        // Send setpoints to each module.
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // After runSetpoint, states might be slightly adjusted for best angles. Log them again.
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /**
     * Characterization mode: run the drive at a specified voltage output to gather data.
     *
     * @param output The voltage to apply to the wheels.
     */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops all drive motion. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stop the drive and turn the modules in an "X" pattern to resist external motion. This is useful for preventing
     * the robot from being easily pushed when idle.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = moduleTranslations[i].getAngle();
        }
        // Reset the kinematics headings so that "X" pattern is established.
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command that runs a quasistatic SysId test in a given direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)) // Run zero voltage first
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction)); // Then run quasistatic test
    }

    /** Returns a command that runs a dynamic SysId test in a given direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.dynamic(direction)); // Then run dynamic test
    }

    /**
     * Returns the module states (angles and velocities) for logging and diagnostics. @AutoLogOutput indicates these
     * outputs are automatically logged.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (angles and distances) of each swerve module. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the current measured chassis speeds based on module states. Useful for feedback and verification. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Returns the characterization positions (in radians) for wheel radius tests. Used during system identification.
     */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of modules in rad/sec for feedforward characterization. */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current estimated robot pose (x, y, rotation) from the pose estimator. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current heading (rotation) of the robot. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the odometry to a given pose. Useful at the start of a match or when known pose is established. */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /**
     * Accepts a vision-based pose measurement to refine the odometry estimate.
     *
     * @param visionRobotPoseMeters Pose from vision system.
     * @param timestampSeconds Timestamp of the vision measurement.
     * @param visionMeasurementStdDevs Standard deviations of vision measurement uncertainty.
     */
    @Override
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed of the robot. */
    public double getMaxLinearSpeedMetersPerSec() {
        return maxSpeedMetersPerSec;
    }

    /** Returns the maximum angular speed (rad/s) of the robot, derived from linear max speed and base radius. */
    public double getMaxAngularSpeedRadPerSec() {
        return maxSpeedMetersPerSec / driveBaseRadius;
    }
}
