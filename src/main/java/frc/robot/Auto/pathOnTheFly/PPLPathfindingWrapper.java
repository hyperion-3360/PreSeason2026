package frc.robot.Auto.pathOnTheFly;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class PPLPathfindingWrapper {
    private RobotConfig config = null;
    private PathPlannerPath createdPath = null;
    private final PathConstraints constraints;

    public PPLPathfindingWrapper(PathConstraints constraints, CommandSwerveDrivetrain drivetrain) {
        this.config = getRobotConfig();
        this.constraints = constraints;

        configureAuto(drivetrain);
    }

    /**
     * Retrieves the robots configs from the pathplanner GUI.
     *
     * @return The robots configs
     */
    private RobotConfig getRobotConfig() {
        try {
            this.config = RobotConfig.fromGUISettings();
            return config;
        } catch (Exception e) {
            // TODO add default robot config if there are no links to the GUI
            e.printStackTrace();
        }
        return null;
    }

    /**
     * Configures the autobuilder which is essential for pathplanner commands.
     *
     * @param drivetrain The currently used drivetrain
     */
    private void configureAuto(CommandSwerveDrivetrain drivetrain) {
        AutoBuilder.configure(
                // returns the current robot pose
                () -> {
                    return drivetrain.getState().Pose;
                },
                // creates a consumer to set the robot pose
                // this is not useful in our case since we have cameras
                pose -> {},
                // returns the current robot relative speed
                () -> {
                    return drivetrain.getState().Speeds;
                },
                // applies a drive request based on pathplanner's calculated speed
                speed -> {
                    var drive =
                            new SwerveRequest.FieldCentric()
                                    .withDeadband(
                                            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
                                                    * 0.1)
                                    .withRotationalDeadband(
                                            RotationsPerSecond.of(
                                                                    Constants.DriveConstants
                                                                            .MAX_ANGULAR_RATE_TELEOP)
                                                            .in(RadiansPerSecond)
                                                    * 0.1) // Add a 10% deadband
                                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
                    drivetrain.applyRequest(
                            () -> {
                                return drive.withVelocityX(speed.vxMetersPerSecond)
                                        .withVelocityY(speed.vyMetersPerSecond)
                                        .withRotationalRate(speed.omegaRadiansPerSecond);
                            });
                },
                // sets the pid constants pathplanner will use
                new PPHolonomicDriveController(new PIDConstants(10), new PIDConstants(10)),
                // gets the configs from the pathplanner GUI
                config,
                // tells autobuilder to flip the path if the alliance is red
                () -> {
                    if (DriverStation.getAlliance().isPresent()) {
                        return DriverStation.getAlliance().get() == Alliance.Blue ? false : true;
                    }
                    return false;
                },
                // sets the swerves as a requirement
                drivetrain);
    }

    /**
     * Schedules a pathfinding command and retrieves it's path
     *
     * @param targetPose The pose to pathfind to
     * @return The generated path
     */
    public PathPlannerPath generatePathFromPathfinding(Pose2d targetPose) {

        AutoBuilder.pathfindToPose(targetPose, constraints);

        Pathfinding.setGoalPosition(targetPose.getTranslation());
        Pathfinding.ensureInitialized();

        createdPath = Pathfinding.getCurrentPath(constraints, null);

        return createdPath;
    }

    public RobotConfig getConfig() {
        return config;
    }
}
