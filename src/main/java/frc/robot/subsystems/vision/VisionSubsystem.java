package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Vision subsystem that manages multiple cameras, provides pose estimates, and handles target
 * selection for alignment.
 */
public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final List<VisionCamera> m_cameras;

    // Simulation
    private VisionSystemSim m_visionSim;
    private Field2d m_visionField;

    // Target tracking
    private int m_targetTagId = -1; // -1 means no target selected
    private double m_lastTargetSeenTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    private TargetPriority m_targetPriority = TargetPriority.BALANCED;
    private boolean m_targetLocked = false; // Prevents target switching during alignment

    // Performance optimization: Cache timestamp to reduce hardware I/O calls during periodic()
    private double m_currentTime = 0;

    // Priority chooser for SmartDashboard
    private final SendableChooser<TargetPriority> m_priorityChooser = new SendableChooser<>();

    /**
     * Creates a new VisionSubsystem.
     *
     * @param drivetrain The swerve drivetrain to provide pose estimates to
     */
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_cameras = new ArrayList<>();

        // Initialize cameras
        setupCameras();

        // Setup simulation if running in sim
        if (RobotBase.isSimulation()) {
            setupSimulation();
        }

        // Setup telemetry
        setupTelemetry();
    }

    /** Configures all cameras. Add/remove cameras here as needed. */
    private void setupCameras() {
        // Camera 1: Limelight 3 - mounted high, looking forward
        // Camera transforms are configured in Constants.VisionConstants
        m_cameras.add(
                new VisionCamera(
                        Constants.VisionConstants.LIMELIGHT_NAME,
                        Constants.VisionConstants.ROBOT_TO_LIMELIGHT,
                        Constants.tagLayout));

        // When you add more cameras, just add them here:
        /*
        Transform3d robotToLL2Right = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(12.25),
                Units.inchesToMeters(-11.125),
                Units.inchesToMeters(7.25)),
            new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(19.7)));
        m_cameras.add(new VisionCamera("limelight-right", robotToLL2Right, Constants.tagLayout));

        Transform3d robotToLL2Left = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(12.25),
                Units.inchesToMeters(11.125),
                Units.inchesToMeters(7.25)),
            new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-19.7)));
        m_cameras.add(new VisionCamera("limelight-left", robotToLL2Left, Constants.tagLayout));
        */
    }

    /** Sets up PhotonVision simulation for AdvantageScope visualization. */
    private void setupSimulation() {
        m_visionSim = new VisionSystemSim("main");
        m_visionSim.addAprilTags(Constants.tagLayout);

        // Enable simulation for all cameras
        for (var camera : m_cameras) {
            camera.enableSimulation(m_visionSim);
        }

        System.out.println("[Vision] Simulation enabled with " + m_cameras.size() + " camera(s)");
    }

    /** Sets up telemetry for SmartDashboard and AdvantageScope. */
    private void setupTelemetry() {
        m_visionField = new Field2d();
        SmartDashboard.putData("Vision/Field", m_visionField);

        // Setup priority chooser
        m_priorityChooser.setDefaultOption("Closest", TargetPriority.CLOSEST);
        m_priorityChooser.addOption("Balanced", TargetPriority.BALANCED);
        m_priorityChooser.addOption("Mostly Closest", TargetPriority.MOSTLY_CLOSEST);
        m_priorityChooser.addOption("Mostly Quality", TargetPriority.MOSTLY_QUALITY);
        m_priorityChooser.addOption("Best Quality", TargetPriority.BEST_QUALITY);
        m_priorityChooser.addOption("Front Facing", TargetPriority.FRONT_FACING);
        m_priorityChooser.addOption("Alliance Only", TargetPriority.ALLIANCE_ONLY);

        SmartDashboard.putData("Vision/Target Priority", m_priorityChooser);
    }

    @Override
    public void periodic() {
        // Cache timestamp once per cycle to reduce hardware I/O calls
        m_currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        // Update priority mode from SmartDashboard chooser
        TargetPriority selectedPriority = m_priorityChooser.getSelected();
        if (selectedPriority != null && selectedPriority != m_targetPriority) {
            m_targetPriority = selectedPriority;
        }

        // Get current robot pose for simulation and telemetry
        var drivetrainState = m_drivetrain.getState();
        Pose2d currentPose = null;
        if (drivetrainState != null && drivetrainState.Pose != null) {
            currentPose = drivetrainState.Pose;
        } else {
            currentPose = new Pose2d(); // Use origin if drivetrain not ready
        }

        // Update simulation if running
        if (RobotBase.isSimulation() && m_visionSim != null) {
            m_visionSim.update(currentPose);
        }

        // Update all cameras - optimized single-loop iteration combining update and telemetry
        // This improves cache locality and reduces loop overhead by processing each camera once
        for (int i = 0; i < m_cameras.size(); i++) {
            var camera = m_cameras.get(i);
            camera.update(currentPose);

            // If camera has a pose estimate, add it to drivetrain
            if (camera.isConnected()) {
                camera.getLatestPose()
                        .ifPresent(
                                pose -> {
                                    m_drivetrain.addVisionMeasurement(
                                            pose.estimatedPose.toPose2d(),
                                            camera.getLatestTimestamp(),
                                            camera.getStdDevs());
                                });
            }

            // Update telemetry for this camera immediately (better cache locality)
            String prefix = "Vision/Camera " + i + " (" + camera.getName() + ")/";
            SmartDashboard.putBoolean(prefix + "Connected", camera.isConnected());
            SmartDashboard.putBoolean(prefix + "Has Target", camera.getBestTarget().isPresent());

            camera.getBestTarget()
                    .ifPresent(
                            target -> {
                                SmartDashboard.putNumber(
                                        prefix + "Target ID", target.getFiducialId());
                                SmartDashboard.putNumber(
                                        prefix + "Distance",
                                        target.getBestCameraToTarget().getTranslation().getNorm());
                                SmartDashboard.putNumber(
                                        prefix + "Ambiguity", target.getPoseAmbiguity());
                            });
        }

        // Update target tracking
        updateTargetTracking();

        // Update telemetry
        updateTelemetry(currentPose);

        // AdvantageKit logging
        Logger.recordOutput("Vision/HasTarget", hasTarget());
        Logger.recordOutput("Vision/TargetID", m_targetTagId);
        Logger.recordOutput("Vision/CameraCount", m_cameras.size());
        Logger.recordOutput("Vision/PriorityMode", m_targetPriority.getDisplayName());

        // Log target pose if available
        getTargetPose()
                .ifPresent(
                        targetPose -> {
                            Logger.recordOutput("Vision/TargetPose", targetPose);
                            Logger.recordOutput("Vision/TargetX", targetPose.getX());
                            Logger.recordOutput("Vision/TargetY", targetPose.getY());
                        });

        // Log angle to target for auto-aim
        Rotation2d angleToTarget = getAngleToTarget();
        Logger.recordOutput("Vision/AngleToTarget", angleToTarget.getRadians());
    }

    /** Updates which AprilTag we're tracking/locked onto. */
    private void updateTargetTracking() {
        // If target is locked, check ALL cameras to see if locked target is visible
        if (m_targetLocked && m_targetTagId != -1) {
            boolean foundInAnyCamera = false;
            for (var camera : m_cameras) {
                Optional<PhotonTrackedTarget> target = camera.getBestTarget();
                if (target.isPresent() && target.get().getFiducialId() == m_targetTagId) {
                    m_lastTargetSeenTime = m_currentTime;
                    foundInAnyCamera = true;
                    break; // Found it, no need to keep searching
                }
            }

            // Only timeout if not found in ANY camera
            if (!foundInAnyCamera
                    && m_currentTime - m_lastTargetSeenTime
                            > Constants.VisionConstants.TARGET_LOCK_TIMEOUT) {
                System.out.println(
                        "[Vision] Lost locked target "
                                + m_targetTagId
                                + " in all cameras, unlocking");
                m_targetLocked = false;
                m_targetTagId = -1;
            }
            return; // Don't update target while locked
        }

        PhotonTrackedTarget bestTarget = null;
        double bestScore = Double.MAX_VALUE;

        // Get robot pose for angle calculations
        var drivetrainState = m_drivetrain.getState();
        Pose2d robotPose =
                (drivetrainState != null && drivetrainState.Pose != null)
                        ? drivetrainState.Pose
                        : new Pose2d();

        // Find the best visible target across all cameras
        for (var camera : m_cameras) {
            Optional<PhotonTrackedTarget> target = camera.getBestTarget();
            if (target.isPresent()) {
                PhotonTrackedTarget currentTarget = target.get();

                // Check alliance filter if ALLIANCE_ONLY mode
                if (m_targetPriority == TargetPriority.ALLIANCE_ONLY) {
                    if (!isAllianceTag(currentTarget.getFiducialId())) {
                        continue; // Skip non-alliance tags
                    }
                }

                // Calculate scoring factors
                double distance = currentTarget.getBestCameraToTarget().getTranslation().getNorm();

                // Filter out targets that are too far away
                if (distance > Constants.VisionConstants.MAX_DETECTION_DISTANCE) {
                    continue; // Skip targets beyond max detection distance
                }
                double ambiguity = currentTarget.getPoseAmbiguity();

                // Calculate angle to target (how far off-center)
                double angle = calculateAngleToTarget(currentTarget, robotPose);

                // Score using configurable weights (lower is better)
                double score =
                        (distance * m_targetPriority.getDistanceWeight())
                                + (ambiguity * m_targetPriority.getAmbiguityWeight())
                                + (angle * m_targetPriority.getAngleWeight());

                if (bestTarget == null || score < bestScore) {
                    bestTarget = currentTarget;
                    bestScore = score;
                }
            }
        }

        // Update target tracking based on best target found
        if (bestTarget != null) {
            m_targetTagId = bestTarget.getFiducialId();
            m_lastTargetSeenTime = m_currentTime;
        } else {
            // Clear target if we haven't seen it recently
            if (m_currentTime - m_lastTargetSeenTime
                    > Constants.VisionConstants.TARGET_LOCK_TIMEOUT) {
                m_targetTagId = -1;
            }
        }
    }

    /** Check if a tag belongs to our alliance */
    private boolean isAllianceTag(int tagId) {
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return true; // If unknown, allow all tags
        }

        // Safe access to alliance
        edu.wpi.first.wpilibj.DriverStation.Alliance allianceColor = alliance.get();

        // Alliance tags: Blue = 6,7,8 | Red = 3,4,5 (adjust for your field layout)
        if (allianceColor == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) {
            return tagId >= 6 && tagId <= 8;
        } else if (allianceColor == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
            return tagId >= 3 && tagId <= 5;
        } else {
            return true; // Unknown alliance - allow all
        }
    }

    /** Calculate angle offset from robot heading to target (in radians) */
    private double calculateAngleToTarget(PhotonTrackedTarget target, Pose2d robotPose) {
        // Get tag pose from field layout
        Optional<Pose2d> tagPose =
                Constants.tagLayout
                        .getTagPose(target.getFiducialId())
                        .map(pose3d -> pose3d.toPose2d());

        if (tagPose.isEmpty()) {
            return 0.0;
        }

        // Safe access to tag pose
        Pose2d tag = tagPose.get();

        // Calculate angle from robot to tag
        double deltaX = tag.getX() - robotPose.getX();
        double deltaY = tag.getY() - robotPose.getY();
        Rotation2d angleToTag = new Rotation2d(deltaX, deltaY);

        // Calculate difference from robot heading
        double angleDifference = Math.abs(angleToTag.minus(robotPose.getRotation()).getRadians());

        // Normalize to 0-π range
        if (angleDifference > Math.PI) {
            angleDifference = 2 * Math.PI - angleDifference;
        }

        return angleDifference;
    }

    /** Updates telemetry data for SmartDashboard and AdvantageScope. */
    private void updateTelemetry(Pose2d robotPose) {
        // Update field widget with robot pose
        m_visionField.setRobotPose(robotPose);

        // Show target tag positions if locked
        if (m_targetTagId != -1) {
            Optional<Pose3d> tagPose = Constants.tagLayout.getTagPose(m_targetTagId);
            tagPose.ifPresent(
                    pose -> {
                        m_visionField.getObject("Target Tag").setPose(pose.toPose2d());
                    });
        } else {
            m_visionField.getObject("Target Tag").setPoses(); // Clear
        }

        // Camera status
        SmartDashboard.putNumber("Vision/Active Cameras", getActiveCameraCount());
        SmartDashboard.putNumber("Vision/Target Tag ID", m_targetTagId);
        SmartDashboard.putBoolean("Vision/Has Target", hasTarget());
        SmartDashboard.putBoolean("Vision/Target Locked", m_targetLocked);

        // Individual camera telemetry is now updated in the main camera loop (periodic method)
        // for better performance and cache locality
    }

    // ========================= PUBLIC API =========================

    /**
     * Gets the currently tracked/locked AprilTag ID.
     *
     * @return The tag ID, or -1 if no target is locked
     */
    public int getTargetTagId() {
        return m_targetTagId;
    }

    /**
     * Checks if we have a valid target locked.
     *
     * @return true if a target is currently tracked
     */
    public boolean hasTarget() {
        return m_targetTagId != -1;
    }

    /**
     * Manually sets which tag to track/align to.
     *
     * @param tagId The AprilTag ID to target
     */
    public void setTargetTag(int tagId) {
        // Bounds check on tag ID to prevent array/lookup errors
        if (tagId < 1 || tagId > 16) {
            System.err.println(
                    "[Vision] Error: Tag ID "
                            + tagId
                            + " out of valid range (1-16). Ignoring request.");
            return;
        }

        // Validate that the tag exists in the field layout
        if (Constants.tagLayout.getTagPose(tagId).isEmpty()) {
            System.err.println(
                    "[Vision] Warning: Tag ID " + tagId + " does not exist in field layout!");
            return;
        }
        m_targetTagId = tagId;
        m_lastTargetSeenTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

    /** Clears the current target. */
    public void clearTarget() {
        m_targetTagId = -1;
        m_targetLocked = false;
    }

    /**
     * Locks the current target to prevent switching during alignment. Use this when starting an
     * alignment command to ensure the robot doesn't switch targets if it gets too close or loses
     * sight temporarily.
     */
    public void lockTarget() {
        if (m_targetTagId != -1) {
            m_targetLocked = true;
            System.out.println("[Vision] Target " + m_targetTagId + " locked");
        }
    }

    /** Unlocks the current target, allowing automatic target switching again. */
    public void unlockTarget() {
        m_targetLocked = false;
        System.out.println("[Vision] Target unlocked");
    }

    /**
     * Checks if target is currently locked.
     *
     * @return true if target is locked
     */
    public boolean isTargetLocked() {
        return m_targetLocked;
    }

    /**
     * Gets the pose of the currently targeted AprilTag.
     *
     * @return Optional containing the tag's pose, or empty if no target
     */
    public Optional<Pose2d> getTargetPose() {
        if (m_targetTagId == -1) {
            return Optional.empty();
        }

        return Constants.tagLayout.getTagPose(m_targetTagId).map(Pose3d::toPose2d);
    }

    /**
     * Calculates the desired robot pose to align with the target tag at a specified distance.
     *
     * @param distanceMeters Distance to maintain from the tag (in meters)
     * @return Optional containing the desired pose, or empty if no target
     */
    public Optional<Pose2d> getAlignmentPose(double distanceMeters) {
        Optional<Pose2d> tagPose = getTargetPose();
        if (tagPose.isEmpty()) {
            return Optional.empty();
        }

        // Calculate pose in front of the tag at the specified distance
        Pose2d tag = tagPose.get();

        // The tag's rotation points away from the tag face
        // We want to face the tag, so we rotate 180 degrees
        Rotation2d faceTagRotation = tag.getRotation().plus(Rotation2d.fromDegrees(180));

        // Calculate position: distanceMeters is from FRONT BUMPER to tag
        // We need to account for robot center to front bumper distance
        double totalDistance =
                distanceMeters + Constants.AutoAlignConstants.ROBOT_CENTER_TO_FRONT_BUMPER;

        // Move back from tag by total distance (so bumper is at desired distance)
        double x = tag.getX() - totalDistance * faceTagRotation.getCos();
        double y = tag.getY() - totalDistance * faceTagRotation.getSin();

        Pose2d alignmentPose = new Pose2d(x, y, faceTagRotation);

        return Optional.of(alignmentPose);
    }

    /**
     * Gets the angle to the currently tracked target.
     *
     * @return Rotation2d representing the angle to face the target
     */
    public Rotation2d getAngleToTarget() {
        Optional<Pose2d> targetPose = getTargetPose();
        if (targetPose.isEmpty()) {
            // No target - return current heading (no change)
            return new Rotation2d();
        }

        // Get current robot pose with null safety
        var drivetrainState = m_drivetrain.getState();
        if (drivetrainState == null || drivetrainState.Pose == null) {
            // Drivetrain not ready - return zero rotation
            return new Rotation2d();
        }
        Pose2d robotPose = drivetrainState.Pose;
        Pose2d target = targetPose.get();

        // Calculate angle from robot to target
        double deltaX = target.getX() - robotPose.getX();
        double deltaY = target.getY() - robotPose.getY();

        return new Rotation2d(deltaX, deltaY);
    }

    /**
     * Gets the distance to the current target tag from any camera that can see it.
     *
     * @return Distance in meters, or empty if target not visible
     */
    private Optional<Double> getTargetDistance() {
        if (m_targetTagId == -1) {
            return Optional.empty();
        }

        for (var camera : m_cameras) {
            Optional<PhotonTrackedTarget> target = camera.getBestTarget();
            if (target.isPresent() && target.get().getFiducialId() == m_targetTagId) {
                double distance = target.get().getBestCameraToTarget().getTranslation().getNorm();
                return Optional.of(distance);
            }
        }

        return Optional.empty();
    }

    /**
     * Checks if the current target is within range for auto-aim operations.
     *
     * @return true if target exists and is within auto-aim range
     */
    public boolean isTargetInAutoAimRange() {
        return getTargetDistance()
                .map(dist -> dist <= Constants.VisionConstants.MAX_AUTO_AIM_DISTANCE)
                .orElse(false);
    }

    /**
     * Checks if the current target is within range for auto-align operations.
     *
     * @return true if target exists and is within auto-align range
     */
    public boolean isTargetInAutoAlignRange() {
        return getTargetDistance()
                .map(dist -> dist <= Constants.VisionConstants.MAX_AUTO_ALIGN_DISTANCE)
                .orElse(false);
    }

    /**
     * Gets the number of currently active (connected) cameras.
     *
     * @return Number of active cameras
     */
    public int getActiveCameraCount() {
        return (int) m_cameras.stream().filter(VisionCamera::isConnected).count();
    }

    /**
     * Gets all cameras managed by this subsystem.
     *
     * @return List of all cameras
     */
    public List<VisionCamera> getCameras() {
        return m_cameras;
    }

    /** For simulation: gets the vision system simulator. */
    public VisionSystemSim getVisionSim() {
        return m_visionSim;
    }

    /**
     * Sets the target priority mode.
     *
     * @param priority The priority mode to use for target selection
     */
    public void setTargetPriority(TargetPriority priority) {
        m_targetPriority = priority;
        System.out.println("[Vision] Target priority changed to: " + priority.getDisplayName());
    }

    /**
     * Gets the current target priority mode.
     *
     * @return Current priority mode
     */
    public TargetPriority getTargetPriority() {
        return m_targetPriority;
    }
}
