// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller, and duty cycle
 * absolute encoder.
 */
public class ModuleIOSpark implements ModuleIO {
    private final Rotation2d zeroRotation;

    // Hardware objects
    private final SparkBase driveSpark;
    private final SparkBase turnSpark;
    private final RelativeEncoder driveEncoder;
    private final CANcoder turnEncoder;

    private final StatusSignal<Angle> turnAbsolutePosition;

    private final StatusSignal<AngularVelocity> turnVelocity;
    // Closed loop controllers
    private final SparkClosedLoopController driveController;
    private final PIDController turnController;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    public ModuleIOSpark(int module) {
        zeroRotation = switch (module) {
            case 0 -> frontLeftZeroRotation;
            case 1 -> frontRightZeroRotation;
            case 2 -> backLeftZeroRotation;
            case 3 -> backRightZeroRotation;
            default -> new Rotation2d();};
        driveSpark = new SparkMax(
                switch (module) {
                    case 0 -> frontLeftDriveCanId;
                    case 1 -> frontRightDriveCanId;
                    case 2 -> backLeftDriveCanId;
                    case 3 -> backRightDriveCanId;
                    default -> 0;
                },
                MotorType.kBrushless);
        turnSpark = new SparkMax(
                switch (module) {
                    case 0 -> frontLeftTurnCanId;
                    case 1 -> frontRightTurnCanId;
                    case 2 -> backLeftTurnCanId;
                    case 3 -> backRightTurnCanId;
                    default -> 0;
                },
                MotorType.kBrushless);

        turnEncoder = new CANcoder(
                switch (module) {
                    case 0 -> CANcoderM0;
                    case 1 -> CANcoderM1;
                    case 2 -> CANcoderM2;
                    case 3 -> CANcoderM3;
                    default -> 0;
                },
                "rio");

        driveEncoder = driveSpark.getEncoder();

        // Configure CANCoder
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.MagnetOffset = zeroRotation.getRotations();
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        turnEncoder.getConfigurator().apply(cancoderConfig);
        turnAbsolutePosition = turnEncoder.getAbsolutePosition();
        turnVelocity = turnEncoder.getVelocity();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnAbsolutePosition, turnVelocity);

        driveController = driveSpark.getClosedLoopController();
        turnController = new PIDController(0.05, 0.0, 0.0);
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        // Configure drive motor
        var driveConfig = new SparkMaxConfig();
        driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(driveMotorCurrentLimit)
                .voltageCompensation(12.0);
        driveConfig
                .encoder
                .positionConversionFactor(driveEncoderPositionFactor)
                .velocityConversionFactor(driveEncoderVelocityFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        driveConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(
                        driveKp, 0.0,
                        driveKd, 0.0);
        driveConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                driveSpark,
                5,
                () -> driveSpark.configure(
                        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

        // Configure turn motor
        var turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(turnInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(turnMotorCurrentLimit)
                .voltageCompensation(12.0);
        turnConfig
                .absoluteEncoder
                .inverted(turnEncoderInverted)
                .positionConversionFactor(turnEncoderPositionFactor)
                .velocityConversionFactor(turnEncoderVelocityFactor)
                .averageDepth(2);
        turnConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
                .pidf(turnKp, 0.0, turnKd, 0.0);
        turnConfig
                .signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                turnSpark,
                5,
                () -> turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // Create odometry queues
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder.getPosition());
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        sparkStickyFault = false;
        ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
        ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
        ifOk(
                driveSpark,
                new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
                (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
        ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        // Update turn inputs
        sparkStickyFault = false;
        inputs.turnPosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
                .minus(zeroRotation);
        inputs.turnVelocityRadPerSec = turnVelocity.getValueAsDouble();

        ifOk(
                turnSpark,
                new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
                (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
                drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveSpark.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnSpark.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
        driveController.setReference(velocityRadPerSec, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint =
                MathUtil.inputModulus(rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
        turnSpark.setVoltage(turnController.calculate(getAngle().getRadians(), rotation.getRadians()));
    }

    public Rotation2d getAngle() {
        if (zeroRotation == null) {
            return new Rotation2d();
        } else {
            return Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
                    .plus(zeroRotation);
        }
    }
}
