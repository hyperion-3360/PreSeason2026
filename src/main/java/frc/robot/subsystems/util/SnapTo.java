package frc.robot.subsystems.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class SnapTo {

    CommandSwerveDrivetrain m_drivetrain;

    public SnapTo(CommandSwerveDrivetrain driveTrain) {

        this.m_drivetrain = driveTrain;
    }

    public double getCurrentAngle() {

        var position = new Rotation2d(m_drivetrain.getState().Pose.getRotation().getDegrees());

        return MathUtil.inputModulus(position.getDegrees(), 0, 360);
    }

    public double setWantedAngle(double angle) {

        var setpoint = new Rotation2d(angle).getDegrees();

        return setpoint;
    }
}
