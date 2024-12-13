package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.List;

public class ArmIOKrakenFOC implements ArmIO {
    // Hardware
    private final TalonFX leaderTalon;
    private final CANcoder absoluteEncoder;

    // Status Signals
    private final StatusSignal<Angle> internalPositionRotations;
    private final StatusSignal<Angle> encoderAbsolutePositionRotations;
    private final StatusSignal<Angle> encoderRelativePositionRotations;
    private final StatusSignal<AngularVelocity> velocityRps;
    private final List<StatusSignal<Voltage>> appliedVoltage;
    private final List<StatusSignal<Current>> supplyCurrent;
    private final List<StatusSignal<Current>> torqueCurrent;
    private final List<StatusSignal<Temperature>> tempCelsius;

    // Control
    private final VoltageOut voltageControl =
            new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
    private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    // Config
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    public ArmIOKrakenFOC() {
        leaderTalon = new TalonFX(ArmConstants.leaderID, "rio");

        absoluteEncoder = new CANcoder(ArmConstants.armEncoderID, "rio");

        // Arm Encoder Configs
        CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
        armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        armEncoderConfig.MagnetSensor.MagnetOffset = Units.radiansToRotations(ArmConstants.kArmZeroCosineOffset);
        absoluteEncoder.getConfigurator().apply(armEncoderConfig, 1.0);

        // Leader motor configs
        config.Slot0.kP = ArmConstants.gains.kP();
        config.Slot0.kI = ArmConstants.gains.kI();
        config.Slot0.kD = ArmConstants.gains.kD();
        config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;

        config.MotorOutput.Inverted = ArmConstants.leaderInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.FeedbackRemoteSensorID = ArmConstants.armEncoderID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.RotorToSensorRatio = ArmConstants.kArmGearRatio;
        config.Feedback.SensorToMechanismRatio = 1.0;
        leaderTalon.getConfigurator().apply(config, 1.0);

        // Status signals
        internalPositionRotations = leaderTalon.getPosition();
        encoderAbsolutePositionRotations = absoluteEncoder.getAbsolutePosition();
        encoderRelativePositionRotations = absoluteEncoder.getPosition();
        velocityRps = leaderTalon.getVelocity();
        appliedVoltage = List.of(leaderTalon.getMotorVoltage());
        supplyCurrent = List.of(leaderTalon.getSupplyCurrent());
        torqueCurrent = List.of(leaderTalon.getTorqueCurrent());
        tempCelsius = List.of(leaderTalon.getDeviceTemp());
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                internalPositionRotations,
                velocityRps,
                appliedVoltage.get(0),
                appliedVoltage.get(1),
                supplyCurrent.get(0),
                supplyCurrent.get(1),
                torqueCurrent.get(0),
                torqueCurrent.get(1),
                tempCelsius.get(0),
                tempCelsius.get(1));

        BaseStatusSignal.setUpdateFrequencyForAll(
                500, encoderAbsolutePositionRotations, encoderRelativePositionRotations);

        // Optimize bus utilization
        leaderTalon.optimizeBusUtilization(0, 1.0);
        absoluteEncoder.optimizeBusUtilization(0, 1.0);
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs.leaderMotorConnected = BaseStatusSignal.refreshAll(
                        internalPositionRotations,
                        velocityRps,
                        appliedVoltage.get(0),
                        supplyCurrent.get(0),
                        torqueCurrent.get(0),
                        tempCelsius.get(0))
                .isOK();
        inputs.followerMotorConnected = BaseStatusSignal.refreshAll(
                        appliedVoltage.get(1), supplyCurrent.get(1), torqueCurrent.get(1), tempCelsius.get(1))
                .isOK();
        inputs.absoluteEncoderConnected = BaseStatusSignal.refreshAll(
                        encoderAbsolutePositionRotations, encoderRelativePositionRotations)
                .isOK();

        inputs.positionRads = Units.rotationsToRadians(internalPositionRotations.getValueAsDouble());
        inputs.absoluteEncoderPositionRads =
                Units.rotationsToRadians(encoderAbsolutePositionRotations.getValueAsDouble())
                        - Units.degreesToRadians(ArmConstants.kArmZeroCosineOffset); // Negate internal offset
        inputs.relativeEncoderPositionRads =
                Units.rotationsToRadians(encoderRelativePositionRotations.getValueAsDouble())
                        - Units.degreesToRadians(ArmConstants.kArmZeroCosineOffset);
        inputs.velocityRadsPerSec =
                Units.rotationsToRadians(velocityRps.getValue().in(RotationsPerSecond));
        inputs.appliedVolts = appliedVoltage.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
        inputs.supplyCurrentAmps = supplyCurrent.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
        inputs.torqueCurrentAmps = torqueCurrent.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
        inputs.tempCelcius =
                tempCelsius.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    }

    @Override
    public void runSetpoint(double setpointRads, Voltage feedforward) {
        leaderTalon.setControl(positionControl
                .withPosition(Units.radiansToRotations(setpointRads))
                .withFeedForward(feedforward.in(Volts)));
    }

    @Override
    public void runVolts(double volts) {
        leaderTalon.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void runCurrent(double amps) {
        leaderTalon.setControl(currentControl.withOutput(amps));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setPID(double p, double i, double d) {
        config.Slot0.kP = p;
        config.Slot0.kI = i;
        config.Slot0.kD = d;
        leaderTalon.getConfigurator().apply(config, 0.01);
    }

    @Override
    public void stop() {
        leaderTalon.setControl(new NeutralOut());
    }
}
