package frc.robot.subsystems.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;

/**
 * Command to automatically align the robot with an AprilTag at a specified distance. Supports two
 * motion profiling modes: - Trapezoidal: Sharp acceleration/deceleration (fast but can be jerky) -
 * S-Curve: Smooth acceleration/deceleration (feels more natural, easier on mechanisms)
 */
public class AlignToTagCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final VisionSubsystem m_vision;
    private final double m_targetDistance;
    private final java.util.function.Supplier<Constants.AutoAlignConstants.MotionProfileType>
            m_profileSupplier;

    // Trapezoidal PID controllers
    private final ProfiledPIDController m_xController;
    private final ProfiledPIDController m_yController;
    private final ProfiledPIDController m_thetaController;

    // S-Curve smoothing filters (applied to trapezoidal output)
    private SlewRateLimiter m_xSmoother;
    private SlewRateLimiter m_ySmoother;
    private SlewRateLimiter m_thetaSmoother;

    // Swerve request for field-centric control
    private final SwerveRequest.FieldCentric m_driveRequest;

    // Target pose
    private Pose2d m_targetPose;
    private boolean m_hasValidTarget;
    private boolean m_alignmentStarted;

    // Throttle console output to reduce spam
    private double m_lastPrintTime;

    // Track adaptive gain mode
    private boolean m_lastWasCloseMode = false;

    /**
     * Creates a new AlignToTagCommand.
     *
     * @param drivetrain The swerve drivetrain
     * @param vision The vision subsystem
     * @param targetDistanceMeters Distance to maintain from the tag (in meters)
     * @param profileSupplier Supplier for the motion profile type
     */
    public AlignToTagCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            double targetDistanceMeters,
            java.util.function.Supplier<Constants.AutoAlignConstants.MotionProfileType>
                    profileSupplier) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        m_targetDistance = targetDistanceMeters;
        m_profileSupplier = profileSupplier;

        // Configure Trapezoidal PID controllers
        m_xController =
                new ProfiledPIDController(
                        Constants.AutoAlignConstants.kP_TRANSLATION,
                        Constants.AutoAlignConstants.kI_TRANSLATION,
                        Constants.AutoAlignConstants.kD_TRANSLATION,
                        new TrapezoidProfile.Constraints(
                                Constants.AutoAlignConstants.MAX_VELOCITY_TRANSLATION,
                                Constants.AutoAlignConstants.MAX_ACCELERATION_TRANSLATION));
        m_xController.setTolerance(Constants.AutoAlignConstants.POSITION_TOLERANCE);

        m_yController =
                new ProfiledPIDController(
                        Constants.AutoAlignConstants.kP_TRANSLATION,
                        Constants.AutoAlignConstants.kI_TRANSLATION,
                        Constants.AutoAlignConstants.kD_TRANSLATION,
                        new TrapezoidProfile.Constraints(
                                Constants.AutoAlignConstants.MAX_VELOCITY_TRANSLATION,
                                Constants.AutoAlignConstants.MAX_ACCELERATION_TRANSLATION));
        m_yController.setTolerance(Constants.AutoAlignConstants.POSITION_TOLERANCE);

        m_thetaController =
                new ProfiledPIDController(
                        Constants.AutoAlignConstants.kP_ROTATION,
                        Constants.AutoAlignConstants.kI_ROTATION,
                        Constants.AutoAlignConstants.kD_ROTATION,
                        new TrapezoidProfile.Constraints(
                                Constants.AutoAlignConstants.MAX_VELOCITY_ROTATION,
                                Constants.AutoAlignConstants.MAX_ACCELERATION_ROTATION));
        m_thetaController.setTolerance(Constants.AutoAlignConstants.ANGLE_TOLERANCE);
        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Create drive request
        m_driveRequest =
                new SwerveRequest.FieldCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // Require both drivetrain and vision to prevent conflicts
        addRequirements(m_drivetrain, m_vision);
    }

    @Override
    public void initialize() {
        // Reset alignment flag
        m_alignmentStarted = false;

        // Lock the target to prevent switching during alignment
        m_vision.lockTarget();

        // Check if target is within auto-align range
        if (!m_vision.isTargetInAutoAlignRange()) {
            System.out.println(
                    "[AlignToTag] Target beyond max auto-align distance ("
                            + Constants.VisionConstants.MAX_AUTO_ALIGN_DISTANCE
                            + "m)!");
            m_hasValidTarget = false;
            m_vision.unlockTarget();
            return;
        }

        // Get the target pose from vision
        var alignmentPose = m_vision.getAlignmentPose(m_targetDistance);

        if (alignmentPose.isEmpty()) {
            System.out.println("[AlignToTag] No valid target found!");
            m_hasValidTarget = false;
            m_vision.unlockTarget();
            return;
        }

        m_targetPose = alignmentPose.get();
        m_hasValidTarget = true;

        // Get current pose with null safety check
        var drivetrainState = m_drivetrain.getState();
        if (drivetrainState == null || drivetrainState.Pose == null) {
            System.err.println(
                    "[AlignToTag] Warning: Drivetrain state or pose is null in initialize!");
            m_hasValidTarget = false;
            m_vision.unlockTarget();
            return;
        }

        Pose2d currentPose = drivetrainState.Pose;

        // Get target pose with null safety
        Optional<Pose2d> targetPoseOpt = m_vision.getTargetPose();
        if (targetPoseOpt.isEmpty()) {
            System.err.println("[AlignToTag] Warning: Target lost during initialization!");
            m_hasValidTarget = false;
            m_vision.unlockTarget();
            return;
        }
        Pose2d targetPose = targetPoseOpt.get();

        // Calculate current distance to tag
        double currentDistanceToTag =
                currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double targetDistanceToTag =
                m_targetPose.getTranslation().getDistance(targetPose.getTranslation());

        // Get selected motion profile type
        Constants.AutoAlignConstants.MotionProfileType profileType = m_profileSupplier.get();

        // Reset PID controllers
        m_xController.reset(currentPose.getX());
        m_yController.reset(currentPose.getY());
        m_thetaController.reset(currentPose.getRotation().getRadians());

        // Initialize S-Curve smoothers if needed
        if (profileType == Constants.AutoAlignConstants.MotionProfileType.EXPONENTIAL) {
            // SlewRateLimiter: limits rate of change (jerk limiting)
            // Lower values = smoother (but slower response)
            // Units: m/s² for translation, rad/s² for rotation
            double translationJerkLimit = 12.0; // m/s² per second (jerk)
            double rotationJerkLimit = 6.0 * Math.PI; // rad/s² per second

            m_xSmoother = new SlewRateLimiter(translationJerkLimit);
            m_ySmoother = new SlewRateLimiter(translationJerkLimit);
            m_thetaSmoother = new SlewRateLimiter(rotationJerkLimit);

            // Reset smoothers to current velocity (start smooth)
            m_xSmoother.reset(0);
            m_ySmoother.reset(0);
            m_thetaSmoother.reset(0);
        }

        // Reset print throttle timer
        m_lastPrintTime = 0;

        System.out.println(
                String.format(
                        "[AlignToTag] Starting alignment to tag %d using %s profiling\n"
                                + "  Current robot pose: %s\n"
                                + "  Target robot pose:  %s\n"
                                + "  Tag pose: %s\n"
                                + "  Current distance to tag: %.2fm\n"
                                + "  Target distance to tag:  %.2fm (should be %.2fm + %.2fm bumper = %.2fm)\n"
                                + "  Need to %s %.2fm",
                        m_vision.getTargetTagId(),
                        profileType.getDisplayName(),
                        currentPose,
                        m_targetPose,
                        targetPose,
                        currentDistanceToTag,
                        targetDistanceToTag,
                        m_targetDistance,
                        Constants.AutoAlignConstants.ROBOT_CENTER_TO_FRONT_BUMPER,
                        m_targetDistance
                                + Constants.AutoAlignConstants.ROBOT_CENTER_TO_FRONT_BUMPER,
                        currentDistanceToTag > targetDistanceToTag ? "move forward" : "BACK UP",
                        Math.abs(currentDistanceToTag - targetDistanceToTag)));

        // Mark that alignment has successfully started
        m_alignmentStarted = true;
    }

    @Override
    public void execute() {
        if (!m_hasValidTarget) {
            return;
        }

        // Get current pose with null safety check
        var drivetrainState = m_drivetrain.getState();
        if (drivetrainState == null || drivetrainState.Pose == null) {
            System.err.println("[AlignToTag] Warning: Drivetrain state or pose is null!");
            return;
        }

        Pose2d currentPose = drivetrainState.Pose;

        // Calculate current distance from bumper to target
        double distanceToTarget =
                currentPose.getTranslation().getDistance(m_targetPose.getTranslation());
        double angleError =
                Math.abs(
                        currentPose.getRotation().getRadians()
                                - m_targetPose.getRotation().getRadians());

        // Print real-time feedback (throttled to every 0.5 seconds)
        double currentTime = System.currentTimeMillis() / 1000.0;
        if (currentTime - m_lastPrintTime >= 0.5) {
            System.out.println(
                    String.format(
                            "[AlignToTag] Distance: %.2fm | Angle Error: %.1f° | Target: %.2fm",
                            distanceToTarget, Math.toDegrees(angleError), m_targetDistance));
            m_lastPrintTime = currentTime;
        }

        // Get selected motion profile type
        Constants.AutoAlignConstants.MotionProfileType profileType = m_profileSupplier.get();

        // Adaptive PID gains based on distance to target
        if (Constants.AutoAlignConstants.ENABLE_ADAPTIVE_GAINS) {
            boolean isClose =
                    distanceToTarget < Constants.AutoAlignConstants.ADAPTIVE_DISTANCE_THRESHOLD;

            // Log when switching between close/far modes
            if (isClose != m_lastWasCloseMode) {
                System.out.println(
                        String.format(
                                "[AlignToTag] Adaptive gains: %s mode (distance: %.2fm)",
                                isClose ? "CLOSE" : "FAR", distanceToTarget));
                m_lastWasCloseMode = isClose;
            }

            if (isClose) {
                // Close to target - use precise gains
                m_xController.setP(Constants.AutoAlignConstants.kP_TRANSLATION_CLOSE);
                m_xController.setD(Constants.AutoAlignConstants.kD_TRANSLATION_CLOSE);
                m_yController.setP(Constants.AutoAlignConstants.kP_TRANSLATION_CLOSE);
                m_yController.setD(Constants.AutoAlignConstants.kD_TRANSLATION_CLOSE);
                m_thetaController.setP(Constants.AutoAlignConstants.kP_ROTATION_CLOSE);
                m_thetaController.setD(Constants.AutoAlignConstants.kD_ROTATION_CLOSE);
            } else {
                // Far from target - use aggressive gains
                m_xController.setP(Constants.AutoAlignConstants.kP_TRANSLATION_FAR);
                m_xController.setD(Constants.AutoAlignConstants.kD_TRANSLATION_FAR);
                m_yController.setP(Constants.AutoAlignConstants.kP_TRANSLATION_FAR);
                m_yController.setD(Constants.AutoAlignConstants.kD_TRANSLATION_FAR);
                m_thetaController.setP(Constants.AutoAlignConstants.kP_ROTATION_FAR);
                m_thetaController.setD(Constants.AutoAlignConstants.kD_ROTATION_FAR);
            }
        }

        // Calculate velocities using PID controllers (always use trapezoidal profiling)
        double xVelocity = m_xController.calculate(currentPose.getX(), m_targetPose.getX());
        double yVelocity = m_yController.calculate(currentPose.getY(), m_targetPose.getY());
        double thetaVelocity =
                m_thetaController.calculate(
                        currentPose.getRotation().getRadians(),
                        m_targetPose.getRotation().getRadians());

        // Apply S-Curve smoothing if enabled
        if (profileType == Constants.AutoAlignConstants.MotionProfileType.EXPONENTIAL) {
            xVelocity = m_xSmoother.calculate(xVelocity);
            yVelocity = m_ySmoother.calculate(yVelocity);
            thetaVelocity = m_thetaSmoother.calculate(thetaVelocity);
        }

        // Apply the calculated velocities to the drivetrain
        m_drivetrain.setControl(
                m_driveRequest
                        .withVelocityX(xVelocity)
                        .withVelocityY(yVelocity)
                        .withRotationalRate(thetaVelocity));
    }

    @Override
    public void end(boolean interrupted) {
        // Unlock the target so it can switch again
        m_vision.unlockTarget();

        // Stop the robot
        m_drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());

        // Only print completion message if alignment actually started
        if (!m_alignmentStarted) {
            // Alignment never started (target too far, no target, etc.) - no message needed
            return;
        }

        if (interrupted) {
            System.out.println("[AlignToTag] Alignment interrupted (target unlocked)");
        } else {
            // Calculate final distance - just show alignment error
            var drivetrainState = m_drivetrain.getState();
            if (drivetrainState != null && drivetrainState.Pose != null && m_targetPose != null) {
                double alignmentError =
                        drivetrainState
                                .Pose
                                .getTranslation()
                                .getDistance(m_targetPose.getTranslation());
                System.out.println(
                        String.format(
                                "[AlignToTag] Alignment complete! Error: %.2fm (target bumper distance: %.2fm)",
                                alignmentError, m_targetDistance));
            } else {
                System.out.println("[AlignToTag] Alignment complete!");
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Finish if no valid target or if we've reached the target pose
        if (!m_hasValidTarget) {
            return true;
        }

        return m_xController.atGoal() && m_yController.atGoal() && m_thetaController.atGoal();
    }

    /**
     * Creates an AlignToTagCommand with the default distance from Constants.
     *
     * @param drivetrain The swerve drivetrain
     * @param vision The vision subsystem
     * @param profileSupplier Supplier for the motion profile type
     * @return A new AlignToTagCommand
     */
    public static AlignToTagCommand withDefaultDistance(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            java.util.function.Supplier<Constants.AutoAlignConstants.MotionProfileType>
                    profileSupplier) {
        return new AlignToTagCommand(
                drivetrain,
                vision,
                Constants.AutoAlignConstants.DEFAULT_ALIGN_DISTANCE,
                profileSupplier);
    }
}
