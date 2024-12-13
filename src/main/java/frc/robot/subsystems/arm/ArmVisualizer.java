// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ArmVisualizer {
    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismLigament2d arm;
    private final String key;

    public ArmVisualizer(String key, Color color) {
        this.key = key;
        mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
        LoggedMechanismRoot2d root = mechanism.getRoot("pivot", 1.0, 0.4);
        arm = new LoggedMechanismLigament2d("arm", ArmConstants.armLength, 20.0, 6, new Color8Bit(color));
        root.append(arm);
    }

    /** Update arm visualizer with current arm angle */
    public void update(double angleRads) {
        // Log Mechanism2d
        arm.setAngle(Rotation2d.fromRadians(angleRads));
        Logger.recordOutput("Arm/Mechanism2d/" + key, mechanism); // FIXME

        // Log 3D poses
        Pose3d pivot = new Pose3d(
                ArmConstants.armOrigin.getX(),
                0.0,
                ArmConstants.armOrigin.getY(),
                new Rotation3d(0.0, -angleRads, 0.0));

        Logger.recordOutput("Arm/Mechanism3d/" + key, pivot);
        // 3D to 2D
        Logger.recordOutput("Arm/Mechanism3d/" + key, pivot.toPose2d());
    }
}
