package frc.robot.Auto.pathOnTheFly;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class PathLogger {
    private boolean pathOnTheFlyCommandFailed = false;
    private PathPlannerPath createdPath = null;
    private Pose2d pointToGo = Pose2d.kZero;
    private boolean pathCreated = false;
    private final PathConstraints constraints;
    private RobotConfig config = null;
    private final CommandSwerveDrivetrain drivetrain;

    public PathLogger(
            RobotConfig config, CommandSwerveDrivetrain drivetrain, PathConstraints constraints) {
        this.config = config;
        this.drivetrain = drivetrain;
        this.constraints = constraints;
    }

    /** Logs useful information called every time a path is created */
    public void log() {

        if (DriverStation.getAlliance().isPresent()) {
            Logger.recordOutput("Current alliance", DriverStation.getAlliance().get());
        }

        Logger.recordOutput("Path status", pathOnTheFlyCommandFailed);

        if (pathCreated) {
            Logger.recordOutput("Pathfinding/path created", pathCreated);
            Logger.recordOutput("Pathfinding constraints", constraints);

            PathPlannerLogging.logActivePath(createdPath);
            PathPlannerLogging.logTargetPose(pointToGo);

            Logger.recordOutput("Current path start state", createdPath.getIdealStartingState());
            Logger.recordOutput("Current path end state", createdPath.getGoalEndState());
            Logger.recordOutput("Current path waypoint count", createdPath.getWaypoints().size());
            Logger.recordOutput("Current path is reversed?", createdPath.isReversed());

            Logger.recordOutput("Current path's length", getPathLength());
            for (int i = 0; i < createdPath.getWaypoints().size(); i++) {
                Logger.recordOutput(
                        "Current path's waypoint " + i, createdPath.getWaypoints().get(i));
            }

            if (createdPath.getEventMarkers().iterator().hasNext()) {
                Logger.recordOutput(
                        "Current event", createdPath.getEventMarkers().iterator().next());
            }
        }
    }

    public void feedNewPathToLogger(PathPlannerPath path) {
        pathCreated = false;
        createdPath = path;
        if (createdPath != null && createdPath.numPoints() != 0) {
            pathCreated = true;

            pointToGo = path.getPathPoses().get(path.getPathPoses().size() - 1);
        }
        log();
    }

    private double getPathLength() {

        PathPlannerTrajectory trajectory =
                createdPath.generateTrajectory(
                        drivetrain.getState().Speeds, createdPath.getInitialHeading(), config);

        List<PathPlannerTrajectoryState> states = trajectory.getStates();

        double totalLengthMeters = 0.0;

        // Starts from the second state because we can't measure a distance between something and
        // nothing
        for (int i = 1; i < states.size(); i++) {
            Pose2d previousPose = states.get(i - 1).pose;
            Pose2d currentPose = states.get(i).pose;

            double segmentDistance =
                    previousPose.getTranslation().getDistance(currentPose.getTranslation());

            totalLengthMeters += segmentDistance;
        }

        return totalLengthMeters;
    }
}
