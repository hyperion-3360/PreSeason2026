package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionCamera {
    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator m_poseEstimator;
    private final String m_name;

    // Simulation
    private PhotonCameraSim m_cameraSim;

    // Latest results
    private PhotonPipelineResult m_latestResult;
    private Optional<EstimatedRobotPose> m_latestPose = Optional.empty();

    // Standard deviations are configured in Constants.VisionConstants

    /**
     * Creates a new VisionCamera.
     *
     * @param cameraName Name of the PhotonVision camera
     * @param robotToCamera Transform from robot center to camera
     * @param fieldLayout AprilTag field layout
     */
    public VisionCamera(
            String cameraName, Transform3d robotToCamera, AprilTagFieldLayout fieldLayout) {
        m_name = cameraName;
        m_camera = new PhotonCamera(cameraName);

        // Setup pose estimator with multi-tag strategy for best accuracy
        m_poseEstimator =
                new PhotonPoseEstimator(
                        fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        System.out.println("[Vision] Camera initialized: " + cameraName);
    }

    /**
     * Enables simulation for this camera.
     *
     * @param visionSim The vision system simulator to add this camera to
     */
    public void enableSimulation(VisionSystemSim visionSim) {
        m_cameraSim = new PhotonCameraSim(m_camera);

        // Add some simulated noise to make it more realistic
        m_cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(m_cameraSim, m_poseEstimator.getRobotToCameraTransform());
    }

    /**
     * Updates the camera and processes new results.
     *
     * @param currentRobotPose Current estimated robot pose for reference
     */
    public void update(Pose2d currentRobotPose) {
        m_latestResult =
                m_camera.getAllUnreadResults().stream()
                        .reduce((first, second) -> second) // Get last result
                        .orElse(new PhotonPipelineResult());

        if (m_latestResult.hasTargets()) {
            // Update reference pose for pose estimator
            m_poseEstimator.setReferencePose(currentRobotPose);
            m_latestPose = m_poseEstimator.update(m_latestResult);
        } else {
            m_latestPose = Optional.empty();
        }
    }

    /**
     * Gets the name of this camera.
     *
     * @return Camera name
     */
    public String getName() {
        return m_name;
    }

    /**
     * Checks if the camera is connected and responding.
     *
     * @return true if camera is connected
     */
    public boolean isConnected() {
        return m_camera.isConnected();
    }

    /**
     * Gets the best target from the latest result.
     *
     * @return Optional containing the best target, or empty if no targets
     */
    public Optional<PhotonTrackedTarget> getBestTarget() {
        if (m_latestResult != null && m_latestResult.hasTargets()) {
            return Optional.of(m_latestResult.getBestTarget());
        }
        return Optional.empty();
    }

    /**
     * Gets the latest pose estimate.
     *
     * @return Optional containing the pose estimate, or empty if no estimate available
     */
    public Optional<EstimatedRobotPose> getLatestPose() {
        return m_latestPose;
    }

    /**
     * Gets the timestamp of the latest result.
     *
     * @return Timestamp in seconds, or current FPGA timestamp if no result available
     */
    public double getLatestTimestamp() {
        if (m_latestResult != null) {
            return m_latestResult.getTimestampSeconds();
        }
        // Return current timestamp instead of 0.0 to avoid timestamp misalignment
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

    /**
     * Gets the standard deviations for pose estimation based on the number of tags seen.
     *
     * @return Standard deviations matrix [x, y, theta]
     */
    public Matrix<N3, N1> getStdDevs() {
        if (m_latestResult != null && m_latestResult.getTargets().size() >= 2) {
            // Multiple tags visible - higher confidence
            return VecBuilder.fill(
                    Constants.VisionConstants.MULTI_TAG_STD_DEV_X,
                    Constants.VisionConstants.MULTI_TAG_STD_DEV_Y,
                    Constants.VisionConstants.MULTI_TAG_STD_DEV_THETA);
        }
        // Single tag - lower confidence
        return VecBuilder.fill(
                Constants.VisionConstants.SINGLE_TAG_STD_DEV_X,
                Constants.VisionConstants.SINGLE_TAG_STD_DEV_Y,
                Constants.VisionConstants.SINGLE_TAG_STD_DEV_THETA);
    }

    /**
     * Gets the underlying PhotonCamera object.
     *
     * @return PhotonCamera instance
     */
    public PhotonCamera getCamera() {
        return m_camera;
    }
}
