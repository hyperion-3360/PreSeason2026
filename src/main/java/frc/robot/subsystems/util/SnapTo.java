package frc.robot.subsystems.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;

public class SnapTo extends Command {
    TunerConstants tunerConstants;

    public boolean snapingTo = false;

    public SnapTo(double angle){
        
        new Rotation2d(tunerConstants.getRotationAsDouble() + angle);
    }

}