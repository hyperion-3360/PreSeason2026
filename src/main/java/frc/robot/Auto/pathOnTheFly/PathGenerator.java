package frc.robot.Auto.pathOnTheFly;

import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;

public class PathGenerator {

    private final PathConstraints constraints;

    public PathGenerator(PathConstraints constraints) {
        this.constraints = constraints;
    }

    /**
     * Transforms the list of poses into waypoints using pathplannerlib
     *
     * @param poseToPathfind The arraylist of pose2d to transform
     * @return A list of bezier curve waypoint
     */
    private List<Waypoint> transformPoseToWaypoint(List<Pose2d> poseToPathfind) {
        return PathPlannerPath.waypointsFromPoses(poseToPathfind);
    }

    /**
     * Generates a path using a list of waypoints. This is mostly used to path around or to
     * something without pathfinding to it. Highest degree of predictability.
     *
     * @param waypointsToPathfind List of bezier curve waypoints Pathplanner is going to use to
     *     generate a path
     * @param startingVelocity The robots ideal starting velocity
     * @param endVelocity The robots ideal end velocity
     * @param startingRotationRadiants The robots ideal starting heading
     * @param endRotationRadiants The robots ideal ending heading
     * @return The generated path
     * @throws Exception If there are not enough waypoints to generate a path
     */
    public PathPlannerPath generatePath(
            Pose2d robotPose,
            List<Pose2d> poses,
            double startingVelocity,
            double endVelocity,
            double startingRotationRadiants,
            double endRotationRadians)
            throws Exception {
        List<Pose2d> tempPosList = new ArrayList<>();
        if (poses.size() < 1) {
            throw new Exception(
                    "not enough waypoints to generate path. A path must contain at least one waypoints.");
        }
        // prevents path failure because robot isn't at the beginning of the path
        tempPosList.add(robotPose);
        tempPosList.addAll(poses);
        return new PathPlannerPath(
                transformPoseToWaypoint(tempPosList),
                constraints,
                new IdealStartingState(
                        startingVelocity, Rotation2d.fromRadians(startingRotationRadiants)),
                new GoalEndState(endVelocity, Rotation2d.fromRadians(endRotationRadians)));
    }

    /**
     * Adds events to an already generated path
     *
     * @param path A generated path
     * @param commandToExecuteOnPath Map storing 3/4 values being the name of the command, the
     *     command itself and the zoning of the command
     */
    public PathPlannerPath eventAdder(
            PathPlannerPath path, List<EventMarker> commandToExecuteOnPath) {

        PathPlannerPath pathWithEvents = null;
        pathWithEvents =
                new PathPlannerPath(
                        PathPlannerPath.waypointsFromPoses(path.getPathPoses()),
                        path.getRotationTargets(),
                        path.getPointTowardsZones(),
                        path.getConstraintZones(),
                        commandToExecuteOnPath,
                        constraints,
                        new IdealStartingState(0, path.getInitialHeading()),
                        new GoalEndState(
                                0,
                                path.getPathPoses()
                                        .get(path.getPathPoses().size() - 1)
                                        .getRotation()),
                        false);
        return pathWithEvents;
    }
}
