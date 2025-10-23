package frc.robot.subsystems.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Command to automatically align the robot with an AprilTag at a specified distance. Uses PID
 * control to smoothly drive to the target pose.
 */
public class AlignToTagCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final VisionSubsystem m_vision;
    private final double m_targetDistance;

    // PID Controllers for driving to pose
    private final ProfiledPIDController m_xController;
    private final ProfiledPIDController m_yController;
    private final ProfiledPIDController m_thetaController;

    // Swerve request for field-centric control
    private final SwerveRequest.FieldCentric m_driveRequest;

    // Target pose
    private Pose2d m_targetPose;
    private boolean m_hasValidTarget;

    /**
     * Creates a new AlignToTagCommand.
     *
     * @param drivetrain The swerve drivetrain
     * @param vision The vision subsystem
     * @param targetDistanceMeters Distance to maintain from the tag (in meters)
     */
    public AlignToTagCommand(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            double targetDistanceMeters) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        m_targetDistance = targetDistanceMeters;

        // Configure PID controllers - values from Constants.AutoAlignConstants
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

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        // Get the target pose from vision
        var alignmentPose = m_vision.getAlignmentPose(m_targetDistance);

        if (alignmentPose.isEmpty()) {
            System.out.println("[AlignToTag] No valid target found!");
            m_hasValidTarget = false;
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
            return;
        }

        Pose2d currentPose = drivetrainState.Pose;

        // Reset and configure PID controllers with current and target poses
        m_xController.reset(currentPose.getX());
        m_yController.reset(currentPose.getY());
        m_thetaController.reset(currentPose.getRotation().getRadians());

        System.out.println(
                "[AlignToTag] Starting alignment to tag "
                        + m_vision.getTargetTagId()
                        + " at pose: "
                        + m_targetPose);
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

        // Calculate velocities using PID controllers
        double xVelocity = m_xController.calculate(currentPose.getX(), m_targetPose.getX());
        double yVelocity = m_yController.calculate(currentPose.getY(), m_targetPose.getY());
        double thetaVelocity =
                m_thetaController.calculate(
                        currentPose.getRotation().getRadians(),
                        m_targetPose.getRotation().getRadians());

        // Apply the calculated velocities to the drivetrain
        m_drivetrain.setControl(
                m_driveRequest
                        .withVelocityX(xVelocity)
                        .withVelocityY(yVelocity)
                        .withRotationalRate(thetaVelocity));
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        m_drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());

        if (interrupted) {
            System.out.println("[AlignToTag] Alignment interrupted");
        } else {
            System.out.println("[AlignToTag] Alignment complete!");
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
     * @return A new AlignToTagCommand
     */
    public static AlignToTagCommand withDefaultDistance(
            CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        return new AlignToTagCommand(
                drivetrain, vision, Constants.AutoAlignConstants.DEFAULT_ALIGN_DISTANCE);
    }
}
