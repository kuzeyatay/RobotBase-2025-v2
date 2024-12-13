package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
    private static final double autoStartAngle = Units.degreesToRadians(80.0);

    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            ArmConstants.kArmGearRatio,
            1.06328,
            ArmConstants.armLength,
            ArmConstants.minAngle,
            ArmConstants.maxAngle,
            false,
            Units.degreesToRadians(0.0));

    private final PIDController controller;
    private double appliedVoltage = 0.0;
    private double positionOffset = 30.0;

    private boolean controllerNeedsReset = false;
    private boolean closedLoop = true;
    private boolean wasNotAuto = true;

    public ArmIOSim() {
        controller = new PIDController(0.0, 0.0, 0.0);
        sim.setState(0.0, 0.0);
        setPosition(0.0);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            controllerNeedsReset = true;
        }

        // Reset at start of auto
        if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
            sim.setState(autoStartAngle, 0.0);
            wasNotAuto = false;
        }
        wasNotAuto = !DriverStation.isAutonomousEnabled();

        sim.update(0.02);

        inputs.positionRads = sim.getAngleRads() + positionOffset;
        inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();
        inputs.appliedVolts = new double[] {appliedVoltage};
        inputs.supplyCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
        inputs.torqueCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
        inputs.tempCelcius = new double[] {0.0};

        // Reset input
        sim.setInputVoltage(0.0);
    }

    @Override
    public void runSetpoint(double setpointRads, Voltage feedforward) {
        if (!closedLoop) {
            controllerNeedsReset = true;
            closedLoop = true;
        }
        if (controllerNeedsReset) {
            controller.reset();
            controllerNeedsReset = false;
        }
        runVolts(controller.calculate(sim.getAngleRads(), setpointRads + positionOffset) + feedforward.in(Volts));
    }

    @Override
    public void runVolts(double volts) {
        closedLoop = false;
        appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(appliedVoltage);
    }

    @Override
    public void setPID(double p, double i, double d) {
        controller.setPID(p, i, d);
    }

    @Override
    public void stop() {
        appliedVoltage = 0.0;
        sim.setInputVoltage(appliedVoltage);
    }

    private void setPosition(double position) {
        positionOffset = position - sim.getAngleRads();
    }
}
