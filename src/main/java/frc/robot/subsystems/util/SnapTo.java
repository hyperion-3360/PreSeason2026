package frc.robot.subsystems.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class SnapTo {

    CommandSwerveDrivetrain m_drivetrain;

    public SnapTo(CommandSwerveDrivetrain driveTrain) {

        this.m_drivetrain = driveTrain;
    }

    public Rotation2d getCurrentAngle() {

        var position =
                new Rotation2d(
                        MathUtil.angleModulus(
                                m_drivetrain.getState().Pose.getRotation().getRadians()));

        return position;
    }
}
