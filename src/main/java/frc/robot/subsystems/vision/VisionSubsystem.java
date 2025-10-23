// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
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
    }

    @Override
    public void periodic() {
        // Get current robot pose for simulation and telemetry
        Pose2d currentPose = m_drivetrain.getState().Pose;

        // Update simulation if running
        if (RobotBase.isSimulation() && m_visionSim != null) {
            m_visionSim.update(currentPose);
        }

        // Update all cameras
        for (var camera : m_cameras) {
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
        }

        // Update target tracking
        updateTargetTracking();

        // Update telemetry
        updateTelemetry(currentPose);
    }

    /** Updates which AprilTag we're tracking/locked onto. */
    private void updateTargetTracking() {
        PhotonTrackedTarget bestTarget = null;
        double bestScore = Double.MAX_VALUE;

        // Find the best visible target across all cameras
        for (var camera : m_cameras) {
            Optional<PhotonTrackedTarget> target = camera.getBestTarget();
            if (target.isPresent()) {
                PhotonTrackedTarget currentTarget = target.get();

                // Score based on distance and ambiguity
                double distance = currentTarget.getBestCameraToTarget().getTranslation().getNorm();
                double ambiguity = currentTarget.getPoseAmbiguity();

                // Lower score is better (closer + less ambiguous = better)
                double score = distance + (ambiguity * 5.0);

                if (bestTarget == null || score < bestScore) {
                    bestTarget = currentTarget;
                    bestScore = score;
                }
            }
        }

        // Update target tracking based on best target found
        if (bestTarget != null) {
            m_targetTagId = bestTarget.getFiducialId();
            m_lastTargetSeenTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        } else {
            // Clear target if we haven't seen it recently
            if (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - m_lastTargetSeenTime
                    > Constants.VisionConstants.TARGET_LOCK_TIMEOUT) {
                m_targetTagId = -1;
            }
        }
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

        // Individual camera data
        for (int i = 0; i < m_cameras.size(); i++) {
            var camera = m_cameras.get(i);
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
        m_targetTagId = tagId;
        m_lastTargetSeenTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

    /** Clears the current target. */
    public void clearTarget() {
        m_targetTagId = -1;
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

        // Calculate position: move back from tag by the desired distance
        double x = tag.getX() - distanceMeters * faceTagRotation.getCos();
        double y = tag.getY() - distanceMeters * faceTagRotation.getSin();

        return Optional.of(new Pose2d(x, y, faceTagRotation));
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

        // Get current robot pose
        Pose2d robotPose = m_drivetrain.getState().Pose;
        Pose2d target = targetPose.get();

        // Calculate angle from robot to target
        double deltaX = target.getX() - robotPose.getX();
        double deltaY = target.getY() - robotPose.getY();

        return new Rotation2d(deltaX, deltaY);
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
}
