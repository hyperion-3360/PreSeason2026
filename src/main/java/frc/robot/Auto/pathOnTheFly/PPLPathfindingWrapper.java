package frc.robot.Auto.pathOnTheFly;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class PPLPathfindingWrapper {
    private RobotConfig config = null;
    private final PathConstraints constraints;
    private final CommandSwerveDrivetrain drivetrain;

    public PPLPathfindingWrapper(PathConstraints constraints, CommandSwerveDrivetrain drivetrain) {
        this.config = getRobotConfig();
        this.constraints = constraints;
        this.drivetrain = drivetrain;

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
        var drive =
                // PPHolonomic gives robot centered velocities
                new SwerveRequest.RobotCentric()
                        // removed deadbands to prevent clipping small pid speeds and stun-locking
                        // the robot
                        .withDriveRequestType(
                                DriveRequestType.Velocity); // Since pp sends vels not vol

        AutoBuilder.configure(
                // returns the current robot pose
                () -> {
                    return drivetrain.getState().Pose;
                },
                // creates a consumer to set the robot pose
                pose -> {
                    drivetrain.resetPose(pose);
                },
                // returns the current robot relative speed
                () -> {
                    return drivetrain.getState().Speeds;
                },
                // applies a drive request based on pathplanner's calculated speed
                speed -> {
                    // updates control request with calculated speeds
                    drive.withVelocityX(speed.vxMetersPerSecond)
                            .withVelocityY(speed.vyMetersPerSecond)
                            .withRotationalRate(speed.omegaRadiansPerSecond);

                    // gives the control request to the controllers
                    drivetrain.setControl(drive);
                },
                // sets the pid constants pathplanner will use
                new PPHolonomicDriveController(new PIDConstants(5), new PIDConstants(5)),
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
        initializePathfinding(targetPose);
        // prevents returning the path before a new one is made
        while (!Pathfinding.isNewPathAvailable()) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
            }
        }
        return new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(
                        Pathfinding.getCurrentPath(
                                        constraints, new GoalEndState(0, targetPose.getRotation()))
                                .getPathPoses()),
                constraints,
                new IdealStartingState(0, drivetrain.getState().Pose.getRotation()),
                new GoalEndState(0, targetPose.getRotation()));
    }

    private void initializePathfinding(Pose2d targetPose) {
        resetPathfinding();
        Pathfinding.setStartPosition(drivetrain.getState().Pose.getTranslation());
        Pathfinding.setGoalPosition(targetPose.getTranslation());
    }

    private void resetPathfinding() {
        // removes any cached paths to prevent reusing the same path twice
        PathPlannerPath.clearCache();
        // creates an impossible path to clear the pathfinding state
        Pathfinding.setStartPosition(Translation2d.kZero);
        Pathfinding.setGoalPosition(new Translation2d(100, 100));
    }

    public RobotConfig getConfig() {
        return config;
    }
}
