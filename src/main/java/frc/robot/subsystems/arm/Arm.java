package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.arm.ArmConstants.gains;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ModeSet;
import frc.robot.ModeSet.Mode;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// Generic Arm class
public class Arm extends SubsystemBase {

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/Gains/kP", gains.kP());
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/Gains/kI", gains.kI());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/Gains/kD", gains.kD());
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/Gains/kS", gains.ffkS());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/Gains/kV", gains.ffkV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Arm/Gains/kA", gains.ffkA());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/Gains/kG", gains.ffkG());

    private static final LoggedTunableNumber maxVelocity =
            new LoggedTunableNumber("Arm/Velocity", ArmConstants.kArmMotionConstraint.maxVelocity);
    private static final LoggedTunableNumber maxAcceleration =
            new LoggedTunableNumber("Arm/Acceleration", ArmConstants.kArmMotionConstraint.maxAcceleration);

    private static final LoggedTunableNumber partialStowUpperLimitDegrees =
            new LoggedTunableNumber("Arm/PartialStowUpperLimitDegrees", 30.0);

    private static final LoggedTunableNumber lowerLimitDegrees =
            new LoggedTunableNumber("Arm/LowerLimitDegrees", ArmConstants.minAngle);
    private static final LoggedTunableNumber upperLimitDegrees =
            new LoggedTunableNumber("Arm/UpperLimitDegrees", ArmConstants.maxAngle);

    private BooleanSupplier disableSupplier = DriverStation::isDisabled;
    public BooleanSupplier coastSupplier = () -> false;
    private BooleanSupplier halfStowSupplier = () -> true;
    private boolean brakeModeEnabled = true;

    private boolean characterizing = false;

    private double goalAngle;

    // Goal Degrees
    @RequiredArgsConstructor
    public enum Goal {
        STOW(() -> 0),
        ANGLE1(new LoggedTunableNumber("Arm/ANGLE1", 45.0)),
        ANGLE2(new LoggedTunableNumber("Arm/ANGLE2", 90.0));

        private final DoubleSupplier armSetpointSupplier;

        private double getRads() {
            return Units.degreesToRadians(armSetpointSupplier.getAsDouble());
        }
    }

    @AutoLogOutput(key = "Arm/AtGoal")
    public boolean atGoal() {
        return EqualsUtil.epsilonEquals(setpointState.position, goalAngle, 1e-3);
    }

    @AutoLogOutput
    @Getter
    @Setter
    private Goal goal = Goal.STOW;

    private ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public TrapezoidProfile profile;
    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

    private final ArmVisualizer measuredVisualizer;
    private final ArmVisualizer setpointVisualizer;
    private final ArmVisualizer goalVisualizer;

    private boolean wasNotAuto = false;

    public ArmFeedforward ff;

    public Arm(ArmIO io) {
        this.io = io;
        io.setBrakeMode(true);
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
        io.setPID(kP.get(), kI.get(), kD.get());
        ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));

        // Set up visualizers
        measuredVisualizer = new ArmVisualizer("Measured", Color.kBlack);
        setpointVisualizer = new ArmVisualizer("Setpoint", Color.kGreen);
        goalVisualizer = new ArmVisualizer("Goal", Color.kBlue);
    }

    public void setOverrides(
            BooleanSupplier disableOverride, BooleanSupplier coastOverride, BooleanSupplier halfStowOverride) {
        disableSupplier = () -> disableOverride.getAsBoolean() || DriverStation.isDisabled();
        coastSupplier = coastOverride;
        halfStowSupplier = halfStowOverride;
    }

    private double getStowAngle() {
        if (DriverStation.isTeleopEnabled() && !halfStowSupplier.getAsBoolean()) {

            return MathUtil.clamp(
                    setpointState.position,
                    Units.degreesToRadians(ArmConstants.minAngle),
                    Units.degreesToRadians(partialStowUpperLimitDegrees.get()));
        } else {
            return Units.degreesToRadians(ArmConstants.minAngle);
        }
    }

    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        setBrakeMode(!coastSupplier.getAsBoolean());

        // Update controllers
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(), () -> ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get()), kS, kG, kV, kA);

        // Check if disabled
        // Also run first cycle of auto to reset arm
        if (disableSupplier.getAsBoolean()
                || (ModeSet.currentMode) == Mode.SIM && DriverStation.isAutonomousEnabled() && wasNotAuto) {
            io.stop();
            // Reset profile when disabled
            setpointState = new TrapezoidProfile.State(inputs.positionRads, 0);
        }
        // Track autonomous enabled
        wasNotAuto = !DriverStation.isAutonomousEnabled();

        // Don't run profile when characterizing, coast mode, or disabled
        if (!characterizing && brakeModeEnabled && !disableSupplier.getAsBoolean()) {
            // Run closed loop
            goalAngle = goal.getRads();
            if (goal == Goal.STOW) {
                goalAngle = getStowAngle();
            }
            setpointState = profile.calculate(
                    0.02,
                    setpointState,
                    new TrapezoidProfile.State(
                            MathUtil.clamp(
                                    goalAngle,
                                    Units.degreesToRadians(lowerLimitDegrees.get()),
                                    Units.degreesToRadians(upperLimitDegrees.get())),
                            0.0));
            if (goal == Goal.STOW
                    && EqualsUtil.epsilonEquals(goalAngle, Units.degreesToRadians(ArmConstants.minAngle))
                    && atGoal()) {
                io.stop();
            } else {
                Angle kMinArmAngle = Radians.of(setpointState.position);
                AngularVelocity kMaxSpeed = RadiansPerSecond.of(setpointState.velocity);
                io.runSetpoint(setpointState.position, ff.calculate(kMinArmAngle, kMaxSpeed));
            }

            goalVisualizer.update(goalAngle);
            Logger.recordOutput("Arm/GoalAngle", goalAngle);
        }

        // Logs
        measuredVisualizer.update(inputs.positionRads);
        setpointVisualizer.update(setpointState.position);

        Logger.recordOutput("Arm/SetpointAngle", setpointState.position);
        Logger.recordOutput("Arm/SetpointVelocity", setpointState.velocity);
        Logger.recordOutput("Arm/currentDeg", Units.radiansToDegrees(inputs.positionRads));
        Logger.recordOutput("Arm/Goal", goal);
    }
}
