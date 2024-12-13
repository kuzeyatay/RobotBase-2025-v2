package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.ModeSet;

public final class ArmConstants {

    public static final double minAngle = 0.0;
    public static final double maxAngle = 270;

    public static final double kSoftLimitReverse = Units.degreesToRadians(minAngle);
    public static final double kSoftLimitForward = Units.degreesToRadians(maxAngle);

    public static final Translation2d armOrigin = new Translation2d(0, 0); // FIXME ????

    public static final double kArmGearRatio = (62.0 / 12.0) * (60.0 / 18.0) * (65.0 / 12.0);

    public static final int leaderID = 13;
    public static final int armEncoderID = 14;

    public static final boolean leaderInverted = true;

    public static final double kArmZeroCosineOffset =
            0.0; // radians to add to converted arm position to get real-world arm position (starts at ~x
    // deg angle)

    public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(2.0, 2.0);

    public static final double armLength = Units.inchesToMeters(20);

    public static final Gains gains =
            switch (ModeSet.currentMode) {
                case SIM -> new Gains(100, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0);
                case REAL, REPLAY -> new Gains(75.0, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0);
            };

    public record Gains(double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
