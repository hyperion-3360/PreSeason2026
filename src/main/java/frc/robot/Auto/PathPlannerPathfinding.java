package frc.robot.Auto;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.*;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/** This class is used to generate and manage pathfinding commands */
public class PathPlannerPathfinding {
    // Making a queue to manage current commands and cancelling them
    private Queue<Command> pathfindingCommands = new LinkedList<>();

    public record EventMarkerSpecs(String commandName, Command command, double... positions) {}

    private PathPlannerPath createdPath = null;
    private Pose2d pointToGo = Pose2d.kZero;
    private boolean pathCreated = false;
    private CommandSwerveDrivetrain drivetrain = null;

    // Necessary path variables
    private PathConstraints constraints =
            PathConstraints.unlimitedConstraints(12.0); // We don't want to limit our speed
    private IdealStartingState startingState;
    private GoalEndState endState;
    private RobotConfig config = null;

    /**
     * Configures the autobuilder to make pathfinding work;
     *
     * @param drivetrain The swerve drivetrain to use
     */
    public PathPlannerPathfinding(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        configureAuto(drivetrain);
    }

    /** Logs useful information called every time a path is created */
    private void log() {

        PathPlannerLogging.logTargetPose(pointToGo);

        if (DriverStation.getAlliance().isPresent()) {
            Logger.recordOutput("Current alliance", DriverStation.getAlliance().get());
        }

        Logger.recordOutput("Pathfinding/path created", pathCreated);
        Logger.recordOutput("Pathfinding constraints", constraints);
    }

    private void logPath(PathPlannerPath createdPath) {
        PathPlannerLogging.logActivePath(createdPath);
        Logger.recordOutput("Current path start state", createdPath.getIdealStartingState());
        Logger.recordOutput("Current path end state", createdPath.getGoalEndState());
        Logger.recordOutput("Current path waypoint count", createdPath.getWaypoints().size());
        Logger.recordOutput("Current path is reversed?", createdPath.isReversed());

        Logger.recordOutput("Current path's length", getPathLength());
        for (int i = 0; i < createdPath.getWaypoints().size(); i++) {
            Logger.recordOutput("Current path waypoints " + i, createdPath.getWaypoints().get(i));
        }

        if (createdPath.getEventMarkers().iterator().hasNext()) {
            Logger.recordOutput("Current event", createdPath.getEventMarkers().iterator().next());
        }
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

    /**
     * Transforms the list of poses into waypoints using pathplannerlib
     *
     * @param poseToPathfind The arraylist of pose2d to transform
     * @return A list of bezier curve waypoint
     */
    private List<Waypoint> transformPoseToWaypoint(ArrayList<Pose2d> poseToPathfind) {
        pointToGo = poseToPathfind.get(poseToPathfind.size() - 1);
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
    private PathPlannerPath generatePath(
            List<Waypoint> waypointsToPathfind,
            double startingVelocity,
            double endVelocity,
            double startingRotationRadiants,
            double endRotationRadians)
            throws Exception {

        if (waypointsToPathfind.size() < 2) {
            throw new Exception(
                    "not enough waypoints to generate path. A path must contain at least two waypoints.");
        }

        startingState =
                new IdealStartingState(
                        startingVelocity, Rotation2d.fromRadians(startingRotationRadiants));
        endState = new GoalEndState(endVelocity, Rotation2d.fromRadians(endRotationRadians));

        createdPath =
                new PathPlannerPath(waypointsToPathfind, constraints, startingState, endState);

        logPath(createdPath);

        return createdPath;
    }

    private List<EventMarker> generateEventMarkers(List<EventMarkerSpecs> commandToExecuteOnPath)
            throws Exception {

        List<EventMarker> eventList = new ArrayList<>();

        for (EventMarkerSpecs specs : commandToExecuteOnPath) {

            if (specs.positions.length == 0) {
                throw new IllegalArgumentException(
                        "Event must specify at least one path position. -1.0 means no zoning,");
            }

            if (specs.positions.length == 1) {

                eventList.add(
                        new EventMarker(specs.commandName, specs.positions[0], specs.command));
            } else if (specs.positions.length == 1) {

                eventList.add(
                        new EventMarker(
                                specs.commandName,
                                specs.positions[0],
                                specs.positions[1],
                                specs.command));
            } else {
                throw new Exception("either more than two position values added per keys or )");
            }
        }

        return eventList;
    }

    /**
     * Adds events to an already generated path
     *
     * @param path A generated path
     * @param commandToExecuteOnPath Map storing 3/4 values being the name of the command, the
     *     command itself and the zoning of the command
     */
    private void generatePathfindingCommand(
            PathPlannerPath path, List<EventMarkerSpecs> commandToExecuteOnPath) {

        PathPlannerPath pathWithEvents = null;
        try {
            pathWithEvents =
                    new PathPlannerPath(
                            path.getWaypoints(),
                            path.getRotationTargets(),
                            path.getPointTowardsZones(),
                            path.getConstraintZones(),
                            generateEventMarkers(commandToExecuteOnPath),
                            constraints,
                            startingState,
                            endState,
                            false);
        } catch (Exception e) {
            pathfindingCommands.offer(Commands.none());
            return;
        }

        pathfindingCommands.offer(AutoBuilder.followPath(pathWithEvents));
    }

    /**
     * Schedules a pathfinding command and immediatly retrieves the created path then cancels the
     * scheduled command. This works because of the queue system.
     *
     * @param targetPose The pose to pathfind to
     * @return The generated path
     */
    private PathPlannerPath generatePathFromPathfinding(Pose2d targetPose) {
        AutoBuilder.pathfindToPose(targetPose, constraints);

        Pathfinding.setGoalPosition(targetPose.getTranslation());
        Pathfinding.ensureInitialized();

        createdPath = Pathfinding.getCurrentPath(constraints, endState);

        return createdPath;
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
    public void configureAuto(CommandSwerveDrivetrain drivetrain) {
        AutoBuilder.configure(
                // returns the current robot pose
                () -> {
                    return drivetrain.getState().Pose;
                },
                // creates a consumer to set the robot pose
                // this is not useful in our case since we have cameras
                new Consumer<Pose2d>() {
                    @Override
                    public void accept(Pose2d t) {}
                },
                // returns the current robot relative speed
                () -> {
                    return drivetrain.getState().Speeds;
                },
                // applies a drive request based on pathplanner's calculated speed
                new Consumer<ChassisSpeeds>() {

                    @Override
                    public void accept(ChassisSpeeds speed) {
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
                    }
                },
                // sets the pid constants pathplanner will use
                new PPHolonomicDriveController(new PIDConstants(10), new PIDConstants(10)),
                // gets the configs from the pathplanner GUI
                getRobotConfig(),
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

    // ==================== PUBLIC API ==================== \\
    /**
     * Follows a generated path depending on the parameters
     *
     * @param pointsToGeneratePath The poses the robot must go to during the path
     * @param startingVelocity The start velocity of the path
     * @param endVelocity The end velocity of the path
     * @param startingRotationRadiants The start rotation of the path
     * @param endRotationRadiants The end rotation of the path
     * @return The queued command
     */
    public Command followGeneratedPath(
            ArrayList<Pose2d> pointsToGeneratePath,
            double startingVelocity,
            double endVelocity,
            double startingRotationRadiants,
            double endRotationRadiants) {
        try {
            generatePath(
                    transformPoseToWaypoint(pointsToGeneratePath),
                    startingVelocity,
                    startingRotationRadiants,
                    endVelocity,
                    endRotationRadiants);
        } catch (Exception e) {
            return Commands.none();
        }
        log();
        if (!pathfindingCommands.isEmpty()) {
            return pathfindingCommands.poll();
        } else return Commands.none();
    }

    /**
     * Follows a generated path depending on the parameters
     *
     * @param pointsToGeneratePath The poses the robot must go to during the path
     * @param startingVelocity The start velocity of the path
     * @param endVelocity The end velocity of the path
     * @param startingRotationRadiants The start rotation of the path
     * @param endRotationRadiants The end rotation of the path
     * @param commandToExecute The trio of values to generate an event
     * @return The queued command
     */
    public Command followGeneratedPath(
            ArrayList<Pose2d> pointsToGeneratePath,
            double startingVelocity,
            double endVelocity,
            double startingRotationRadiants,
            double endRotationRadiants,
            List<EventMarkerSpecs> commandToExecute) {

        try {
            generatePath(
                    transformPoseToWaypoint(pointsToGeneratePath),
                    startingVelocity,
                    startingRotationRadiants,
                    endVelocity,
                    endRotationRadiants);
        } catch (Exception e) {
            return Commands.none();
        }

        generatePathfindingCommand(createdPath, commandToExecute);

        log();

        if (!pathfindingCommands.isEmpty()) {
            return pathfindingCommands.poll();
        } else return Commands.none();
    }

    /**
     * Follows a generated path depending on the parameters
     *
     * @param pose The pose to pathfind to
     * @param commandToExecute The trio of values to generate an event
     * @return The queued command
     */
    public Command followGeneratedPath(Pose2d pose, List<EventMarkerSpecs> commandToExecute) {
        generatePathfindingCommand(generatePathFromPathfinding(pose), commandToExecute);

        log();

        if (!pathfindingCommands.isEmpty()) {
            return pathfindingCommands.poll();
        } else return Commands.none();
    }

    /**
     * Follows a generated path depending on the parameters
     *
     * @param pose The pose to pathfind to
     * @return The queued command
     */
    public Command followGeneratedPath(Pose2d pose) {
        generatePathFromPathfinding(pose);

        log();

        if (!pathfindingCommands.isEmpty()) {
            return pathfindingCommands.poll();
        } else return Commands.none();
    }
}
