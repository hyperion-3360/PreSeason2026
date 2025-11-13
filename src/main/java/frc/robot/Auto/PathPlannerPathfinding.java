package frc.robot.Auto;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;

public class PathPlannerPathfinding {
   private PathPlannerPath createdPath = null;
   private Pose2d pointToGo = null;
   private boolean pathCreated = false;

    private void log() {
        PathPlannerLogging.logActivePath(createdPath);
        PathPlannerLogging.logTargetPose(pointToGo);
        Logger.recordOutput("Pathfinding/path created", pathCreated);
        Logger.recordOutput("Pathplanner/following path", createdPath.toString());
    }
    // ==================== PUBLIC API ==================== \\

    public PathPlannerPath getCreatedPath(ArrayList<Pose2d> pointsToGeneratePath) {
        if (pathCreated) {
            return createdPath;
        }else
        // TODO change null to a real return
        return null;
    }
}
