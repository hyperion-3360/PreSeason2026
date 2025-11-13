package frc.robot.subsystems.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;

public class SnapTo extends Command {
    TunerConstants tunerConstants;

    public double getCurrentAngle() {

        var position = new Rotation2d(tunerConstants.getRotationAsDouble()).getDegrees();

        return position;
    }

    public double setWantedAngle(double angle) {

        var setpoint = new Rotation2d(angle).getDegrees();

        return setpoint;
    }
}
