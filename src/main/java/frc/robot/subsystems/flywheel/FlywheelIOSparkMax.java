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

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;

// example flywheels with two motors
public class FlywheelIOSparkMax implements FlywheelIO {
    private static final double GEAR_RATIO = 1;

    private final SparkMax leader = new SparkMax(0, MotorType.kBrushless);
    private final SparkMax follower = new SparkMax(1, MotorType.kBrushless);
    private final RelativeEncoder encoder = leader.getEncoder();
    private final SparkClosedLoopController pid = leader.getClosedLoopController();
    private final SparkMaxConfig config = new SparkMaxConfig();

    public FlywheelIOSparkMax() {

        config.voltageCompensation(12.0).smartCurrentLimit(80);
        tryUntilOk(
                leader,
                5,
                () -> leader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        tryUntilOk(
                follower,
                5,
                () -> follower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
        inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
        inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
        inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
    }

    @Override
    public void setVoltage(double volts) {
        leader.setVoltage(volts);
    }

    @Override
    public void setVelocity(double velocityRadPerSec, Voltage ffVolts) {
        pid.setReference(
                Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
                ControlType.kVelocity,
                0,
                ffVolts.in(Volts),
                ArbFFUnits.kVoltage);
    }

    @Override
    public void stop() {
        leader.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        /*
         * Configure the closed loop controller. We want to make sure we set the
         * feedback sensor as the primary encoder.
         */
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for velocity control in slot 1
                .p(kP, ClosedLoopSlot.kSlot1)
                .i(kI, ClosedLoopSlot.kSlot1)
                .d(kD, ClosedLoopSlot.kSlot1)
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
    }
}
