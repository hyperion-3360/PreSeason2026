package frc.robot.Auto.pathOnTheFly;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Auto.pathOnTheFly.EventBuilder.EventMarkerSpecs;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.List;

public class PathCommandManager {
    private PathPlannerPath createdPath = null;

    private final EventBuilder m_eventBuilder;
    private final PathGenerator m_pathGenerator;
    private final PathLogger m_pathLogger;
    private final PPLPathfindingWrapper m_pathfindingWrapper;

    public record PathGenerationSpecs(
            List<Pose2d> posesToGo,
            double startVel,
            double endVel,
            double startRot,
            double endRot) {}

    // Necessary path variables
    private final PathConstraints constraints =
            PathConstraints.unlimitedConstraints(12.0); // We don't want to limit our speed

    public PathCommandManager(CommandSwerveDrivetrain drivetrain) {
        m_eventBuilder = new EventBuilder();
        m_pathGenerator = new PathGenerator(constraints);
        m_pathfindingWrapper = new PPLPathfindingWrapper(constraints, drivetrain);
        m_pathLogger = new PathLogger(m_pathfindingWrapper.getConfig(), drivetrain, constraints);
    }

    /**
     * Follows a generated path depending on the parameters
     *
     * @param parameters The following path generation parameters :
     *     <ul>
     *       <li>The poses the robot must go to during the path
     *       <li>The start velocity of the path
     *       <li>The end velocity of the path
     *       <li>The start rotation of the path
     *       <li>The end rotation of the path
     *     </ul>
     *
     * @return The Autobuilder path-following or nothing if path creation failed
     */
    public Command followGeneratedPath(PathGenerationSpecs parameters) {
        try {
            createdPath =
                    m_pathGenerator.generatePath(
                            parameters.posesToGo,
                            parameters.startVel,
                            parameters.endVel,
                            parameters.startRot,
                            parameters.endRot);
        } catch (Exception e) {
            return Commands.none();
        }
        m_pathLogger.feedNewPathToLogger(createdPath);

        return AutoBuilder.followPath(createdPath);
    }

    /**
     * Follows a generated path depending on the parameters and adds events to that path
     *
     * @param parameters The following path generation parameters :
     *     <ul>
     *       <li>The poses the robot must go to during the path
     *       <li>The start velocity of the path
     *       <li>The end velocity of the path
     *       <li>The start rotation of the path
     *       <li>The end rotation of the path
     *     </ul>
     *
     * @param commandToExecute The trio of values to generate an event
     *     <ul>
     *       <li>The name of the command
     *       <li>The command itself
     *       <li>The zoning of the command along the path
     *     </ul>
     *
     * @return The Autobuilder path-following or nothing if path creation failed
     */
    public Command followGeneratedPath(
            PathGenerationSpecs parameters, List<EventMarkerSpecs> commandToExecute) {

        try {
            createdPath =
                    m_pathGenerator.eventAdder(
                            m_pathGenerator.generatePath(
                                    parameters.posesToGo,
                                    parameters.startVel,
                                    parameters.endVel,
                                    parameters.startRot,
                                    parameters.endRot),
                            m_eventBuilder.generateEventMarkers(commandToExecute));
        } catch (Exception e) {
            return Commands.none();
        }

        m_pathLogger.feedNewPathToLogger(createdPath);
        m_pathLogger.log();

        return AutoBuilder.followPath(createdPath);
    }

    /**
     * Follows a generated path depending on the parameters
     *
     * @param pose The pose to pathfind to
     * @param commandsToExecute The trio of values to generate an event
     * @return The queued command
     */
    public Command pathfindToPose(Pose2d pose, List<EventMarkerSpecs> commandsToExecute) {
        try {
            createdPath =
                    m_pathGenerator.eventAdder(
                            m_pathfindingWrapper.generatePathFromPathfinding(pose),
                            m_eventBuilder.generateEventMarkers(commandsToExecute));
        } catch (Exception e) {
            return Commands.none();
        }

        m_pathLogger.feedNewPathToLogger(createdPath);
        m_pathLogger.log();

        return AutoBuilder.followPath(createdPath);
    }

    /**
     * Follows a generated path depending on the parameters `
     *
     * @param pose The pose to pathfind to
     * @return The queued command
     */
    public Command pathfindToPose(Pose2d pose) {

        createdPath = m_pathfindingWrapper.generatePathFromPathfinding(pose);
        m_pathLogger.feedNewPathToLogger(createdPath);
        m_pathLogger.log();

        return AutoBuilder.followPath(createdPath);
    }
}
