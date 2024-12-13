package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

/**
 * DriveConstants class contains all the constants and parameters related to the robot’s swerve drivetrain
 * configuration. These values include physical dimensions, gear ratios, motor specifications, PID constants, and
 * simulation parameters.
 */
public class DriveConstants {
    // The maximum driving speed of the robot in meters per second.
    public static final double maxSpeedMetersPerSec = 4.8;

    // Frequency for odometry updates (times per second).
    public static final double odometryFrequency = 100.0; // Hz

    // Distance between left and right wheels (the robot’s track width), converted from inches to meters.
    // FIXME means these values might need to be verified or updated.
    public static final double trackWidth = Units.inchesToMeters(26.5); // distance between left & right wheels

    // Distance between front and back wheels (the robot’s wheelbase), also in meters.
    public static final double wheelBase = Units.inchesToMeters(26.5);

    // The radius from the center of the robot to any of the wheels (half diagonal of the drive base).
    // Used in kinematic calculations, especially for turning.
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

    // Positions of each swerve module relative to the robot center.
    // Front-left, front-right, back-left, back-right.
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0), // Front-Left module
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // Front-Right module
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // Back-Left module
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // Back-Right module
    };

    // Zeroed rotation offsets for each swerve module.
    // Typically these are determined at setup (where wheel "0°" position is).
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

    // CAN IDs for various drivetrain components.
    public static final int pigeonCanId = 20; // Gyro (Pigeon) CAN ID

    // Motor controller CAN IDs for drive and turn motors on each module.
    // Front-left, back-left, front-right, back-right pattern.
    // Drive motors:
    public static final int frontLeftDriveCanId = 1;
    public static final int backLeftDriveCanId = 5;
    public static final int frontRightDriveCanId = 3;
    public static final int backRightDriveCanId = 7;
    // Turn motors:
    public static final int frontLeftTurnCanId = 2;
    public static final int backLeftTurnCanId = 6;
    public static final int frontRightTurnCanId = 4;
    public static final int backRightTurnCanId = 8;

    // CANcoders:
    public static final int CANcoderM0 = 10;
    public static final int CANcoderM1 = 11;
    public static final int CANcoderM2 = 9;
    public static final int CANcoderM3 = 12;

    // Current limits to prevent motor damage or electrical issues.
    public static final int driveMotorCurrentLimit = 50;

    // Wheel radius in meters (converted from inches).
    public static final double wheelRadiusMeters = Units.inchesToMeters(2);

    // The total gear reduction for the drive motors.
    // This calculation combines several gear stages: (50/14) * (16/28) * (45/15) = L3 stage gearing.
    public static final double driveMotorReduction = ((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0));

    // Representing the drive motor gearbox. A single NEO motor per wheel.
    public static final DCMotor driveGearbox = DCMotor.getNEO(1);

    // Encoder position factor: Converts from motor rotor rotations to wheel radians.
    // Since encoders read motor rotations, we must account for gear reduction and wheel circumference.
    public static final double driveEncoderPositionFactor =
            2 * Math.PI / driveMotorReduction; // (motor rotations) * (2*pi/driveReduction) = wheel radians

    // Encoder velocity factor: Converts from motor rotor RPM to wheel radians per second.
    public static final double driveEncoderVelocityFactor =
            (2 * Math.PI) / 60.0 / driveMotorReduction; // RPM to rad/s accounting for gear reduction

    // Drive PID constants for closed-loop control. These are initially zero or small values until tuned.
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;

    // Drive feedforward constants, used for motion control (Ks = static friction, Kv = velocity proportional).
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;

    // Simulation-specific PID constants for drive motors when simulating the robot.
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;

    // Simulation feedforward constants for drive motors.
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration: no inversion by default.
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 20;

    // Gear reduction for the turning motor.
    // A larger reduction for turning typically means finer angle control but slower rotation.
    public static final double turnMotorReduction = (150.0 / 7.0);

    // Using a single NEO for the turn axis as well.
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);

    // Turn encoder configuration:
    // turnEncoderInverted indicates if the encoder direction is reversed.
    public static final boolean turnEncoderInverted = true;

    // Conversion factors: from rotations to radians for turning.
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // 1 rotation = 2π radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM to rad/s

    // Turn PID constants for angle control.
    public static final double turnKp = 2.0; // FIXME
    public static final double turnKd = 0.0;

    // Simulation-specific turn PID values.
    public static final double turnSimP = 8.0;
    public static final double turnSimD = 0.0;

    // Limits for turn PID controller input range. Usually 0 to 2π for a continuous rotating module.
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // PathPlanner configuration values:
    // Robot mass, moment of inertia (rotational inertia), and wheel coefficient of friction.
    // FIXME means these might need real-world measurements for accuracy.
    public static final double robotMassKg = 74.088; // total mass of the robot in kg
    public static final double robotMOI = 6.883; // moment of inertia about the vertical axis of the robot
    public static final double wheelCOF = 1.2; // coefficient of friction for the wheels (test value)

    // RobotConfig is used by PathPlanner to simulate and plan paths with correct dynamics.
    public static final RobotConfig ppConfig = new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                    wheelRadiusMeters,
                    maxSpeedMetersPerSec,
                    wheelCOF,
                    driveGearbox.withReduction(driveMotorReduction),
                    driveMotorCurrentLimit,
                    1 // One drive motor per module
                    ),
            moduleTranslations // Positions of modules for path planning
            );

    // Simulation configuration for the drivetrain using MapleSim or a similar simulator.
    // Configures module placements, mass, gyro, and module simulation parameters.
    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
            .withCustomModuleTranslations(moduleTranslations)
            .withRobotMass(Kilogram.of(robotMassKg))
            .withGyro(GyroSimulation.getPigeon2()) // Simulate a Pigeon2 gyro
            .withSwerveModule(() -> new SwerveModuleSimulation(
                    driveGearbox,
                    turnGearbox,
                    driveMotorReduction,
                    turnMotorReduction,
                    Volts.of(0.1), // Drive friction voltage
                    Volts.of(0.1), // Turning friction voltage
                    Meters.of(wheelRadiusMeters),
                    KilogramSquareMeters.of(0.02), // Wheel inertia approximation
                    wheelCOF));
}
