package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.gamepiece.GamePieceManager;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Master coordinator for the entire robot. Orchestrates vision, drivetrain, and game piece
 * manipulation based on player inputs and robot state.
 *
 * <p>This is the "brain" that decides what the robot should be doing at any given moment.
 */
public class RobotOrchestrator extends SubsystemBase {
    // Subsystems
    private final CommandSwerveDrivetrain m_drivetrain;
    private final VisionSubsystem m_vision;
    private final GamePieceManager m_gamePiece;

    // Current master state
    private RobotState m_currentState = RobotState.TELEOP_MANUAL;
    private double m_stateEntryTime = 0;

    // Player inputs (set from RobotContainer)
    private boolean m_requestAutoAlignPiece = false;
    private boolean m_requestIntake = false;
    private boolean m_requestAutoAlignScore = false;
    private boolean m_requestScore = false;
    private boolean m_requestEject = false;
    private boolean m_requestManualOverride = false;

    // Flags from subsystems
    private boolean m_visionHasTarget = false;
    private boolean m_isAligned = false;
    private boolean m_hasPiece = false;
    private boolean m_mechanismsReady = false;

    public RobotOrchestrator(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            GamePieceManager gamePiece) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        m_gamePiece = gamePiece;

        enterState(RobotState.TELEOP_MANUAL);
    }

    @Override
    public void periodic() {
        // Read subsystem states
        updateSubsystemStates();

        // Run master state machine
        RobotState nextState = updateStateMachine();

        if (nextState != m_currentState) {
            enterState(nextState);
        }

        // Coordinate subsystems based on current state
        coordinateSubsystems();

        // Telemetry
        updateTelemetry();
    }

    /**
     * Master state machine logic. Coordinates all subsystems based on player inputs and sensor
     * data.
     */
    private RobotState updateStateMachine() {
        // Emergency stop always takes priority
        if (m_requestManualOverride) {
            return RobotState.TELEOP_MANUAL;
        }

        switch (m_currentState) {
            case TELEOP_MANUAL:
                // Player requests auto-align to game piece
                if (m_requestAutoAlignPiece && m_visionHasTarget) {
                    return RobotState.AUTO_ALIGNING_TO_PIECE;
                }
                // Player requests intake
                if (m_requestIntake) {
                    return RobotState.SEARCHING_FOR_PIECE;
                }
                // Player requests score (and has piece)
                if (m_requestScore && m_hasPiece) {
                    return RobotState.AUTO_ALIGNING_TO_SCORE;
                }
                break;

            case SEARCHING_FOR_PIECE:
                // Found piece with vision, auto-align
                if (m_visionHasTarget && m_requestAutoAlignPiece) {
                    return RobotState.AUTO_ALIGNING_TO_PIECE;
                }
                // Piece acquired without vision
                if (m_hasPiece) {
                    return RobotState.HOLDING_PIECE;
                }
                // Player cancelled
                if (!m_requestIntake) {
                    return RobotState.TELEOP_MANUAL;
                }
                break;

            case AUTO_ALIGNING_TO_PIECE:
                // Aligned and close enough to intake
                if (m_isAligned) {
                    return RobotState.INTAKING_PIECE;
                }
                // Lost target
                if (!m_visionHasTarget) {
                    return RobotState.SEARCHING_FOR_PIECE;
                }
                // Player cancelled
                if (!m_requestAutoAlignPiece && !m_requestIntake) {
                    return RobotState.TELEOP_MANUAL;
                }
                break;

            case INTAKING_PIECE:
                // Successfully acquired piece
                if (m_hasPiece) {
                    return RobotState.HOLDING_PIECE;
                }
                // Player cancelled or timeout
                if (!m_requestIntake) {
                    return RobotState.TELEOP_MANUAL;
                }
                break;

            case HOLDING_PIECE:
                // Player requests score
                if (m_requestScore) {
                    return RobotState.AUTO_ALIGNING_TO_SCORE;
                }
                // Player requests eject
                if (m_requestEject) {
                    // Let GamePieceManager handle ejection
                    m_gamePiece.requestEject(true);
                    return RobotState.TELEOP_MANUAL;
                }
                // Lost piece somehow
                if (!m_hasPiece) {
                    System.err.println("[Orchestrator] Lost game piece unexpectedly!");
                    return RobotState.TELEOP_MANUAL;
                }
                break;

            case AUTO_ALIGNING_TO_SCORE:
                // Aligned to scoring position
                if (m_isAligned) {
                    return RobotState.PREPARING_TO_SCORE;
                }
                // Lost target
                if (!m_visionHasTarget) {
                    return RobotState.HOLDING_PIECE;
                }
                // Player cancelled
                if (!m_requestScore) {
                    return RobotState.HOLDING_PIECE;
                }
                break;

            case PREPARING_TO_SCORE:
                // Mechanisms in position
                if (m_mechanismsReady) {
                    return RobotState.READY_TO_SCORE;
                }
                // Player cancelled
                if (!m_requestScore) {
                    return RobotState.HOLDING_PIECE;
                }
                break;

            case READY_TO_SCORE:
                // Player confirms score
                if (m_requestScore) {
                    return RobotState.SCORING;
                }
                // Player released button
                if (!m_requestScore) {
                    return RobotState.HOLDING_PIECE;
                }
                break;

            case SCORING:
                // Piece released
                if (!m_hasPiece) {
                    return RobotState.TELEOP_MANUAL;
                }
                // Timeout (1 second)
                if (Timer.getFPGATimestamp() - m_stateEntryTime > 1.0) {
                    return RobotState.TELEOP_MANUAL;
                }
                break;

            case EMERGENCY_STOP:
                // Manual reset required
                break;
        }

        return m_currentState;
    }

    /** Called when entering a new state */
    private void enterState(RobotState newState) {
        System.out.println("[Orchestrator] State: " + m_currentState + " → " + newState);
        m_currentState = newState;
        m_stateEntryTime = Timer.getFPGATimestamp();

        // State entry actions
        switch (newState) {
            case TELEOP_MANUAL:
                // Release control of all subsystems
                break;

            case SEARCHING_FOR_PIECE:
                // Enable vision, look for game pieces
                m_gamePiece.requestIntake(true);
                break;

            case AUTO_ALIGNING_TO_PIECE:
                // Start auto-align command to game piece
                // This would be a command scheduled here
                break;

            case INTAKING_PIECE:
                // Stop moving, run intake
                m_gamePiece.requestIntake(true);
                break;

            case HOLDING_PIECE:
                // Stop intake
                m_gamePiece.requestIntake(false);
                break;

            case AUTO_ALIGNING_TO_SCORE:
                // Start auto-align to scoring position
                // Vision looks for AprilTag
                break;

            case PREPARING_TO_SCORE:
                // Command mechanisms to scoring position
                m_gamePiece.requestScore(true);
                break;

            case READY_TO_SCORE:
                // Rumble controller, ready to release
                break;

            case SCORING:
                // Release game piece
                m_gamePiece.requestScore(true);
                break;

            case EMERGENCY_STOP:
                // Stop everything
                m_gamePiece.forceIdle();
                break;
        }
    }

    /** Coordinate subsystems based on current state */
    private void coordinateSubsystems() {
        switch (m_currentState) {
            case TELEOP_MANUAL:
                // Driver has full control - don't interfere
                // Auto-aim can still be active if toggled
                break;

            case SEARCHING_FOR_PIECE:
                // Vision actively looking for game pieces
                // Driver can still move
                break;

            case AUTO_ALIGNING_TO_PIECE:
                // Vision controls rotation, driver controls translation
                // Similar to auto-aim but for game pieces
                break;

            case INTAKING_PIECE:
                // Intake running
                // Driver should stop or slow down
                break;

            case HOLDING_PIECE:
                // Normal driving with piece
                // Maybe limit max speed for stability
                break;

            case AUTO_ALIGNING_TO_SCORE:
                // Vision aligns to AprilTag
                // Full auto-align command running
                break;

            case PREPARING_TO_SCORE:
                // Mechanisms moving to position
                // Driver should be stationary
                break;

            case READY_TO_SCORE:
                // Everything ready, waiting for confirmation
                break;

            case SCORING:
                // Releasing piece
                break;

            case EMERGENCY_STOP:
                // All stop
                break;
        }
    }

    /** Read states from all subsystems */
    private void updateSubsystemStates() {
        m_visionHasTarget = m_vision.hasTarget();
        m_hasPiece = m_gamePiece.hasGamePiece();
        m_mechanismsReady = m_gamePiece.isReadyToScore();

        // Check if aligned (would use AlignToTagCommand.isFinished() in real impl)
        m_isAligned = false; // TODO: Implement based on your alignment logic
    }

    /** Telemetry */
    private void updateTelemetry() {
        SmartDashboard.putString("Robot/State", m_currentState.getDisplayName());
        SmartDashboard.putBoolean("Robot/Has Piece", m_hasPiece);
        SmartDashboard.putBoolean("Robot/Vision Target", m_visionHasTarget);
        SmartDashboard.putBoolean(
                "Robot/Ready to Score", m_currentState == RobotState.READY_TO_SCORE);

        Logger.recordOutput("Robot/State", m_currentState.toString());
        Logger.recordOutput("Robot/HasPiece", m_hasPiece);
        Logger.recordOutput("Robot/VisionTarget", m_visionHasTarget);
    }

    // ==================== PUBLIC API (called from RobotContainer) ====================

    /** Request auto-align to game piece */
    public void requestAutoAlignToPiece(boolean enable) {
        m_requestAutoAlignPiece = enable;
    }

    /** Request intake */
    public void requestIntake(boolean enable) {
        m_requestIntake = enable;
    }

    /** Request auto-align to scoring position */
    public void requestAutoAlignToScore(boolean enable) {
        m_requestAutoAlignScore = enable;
    }

    /** Request score */
    public void requestScore(boolean enable) {
        m_requestScore = enable;
    }

    /** Request eject */
    public void requestEject(boolean enable) {
        m_requestEject = enable;
    }

    /** Manual override - return to teleop */
    public void requestManualOverride() {
        m_requestManualOverride = true;
    }

    /** Get current state */
    public RobotState getCurrentState() {
        return m_currentState;
    }

    /** Check if robot is in auto mode (not manual control) */
    public boolean isInAutoMode() {
        return m_currentState != RobotState.TELEOP_MANUAL;
    }

    /** Emergency stop */
    public void emergencyStop() {
        enterState(RobotState.EMERGENCY_STOP);
    }
}
