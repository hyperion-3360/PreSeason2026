package frc.robot.subsystems.gamepiece;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Manages the state machine for game piece manipulation. Coordinates between sensors, player
 * inputs, and mechanism states.
 */
public class GamePieceManager extends SubsystemBase {
    // Current state
    private GamePieceState m_currentState = GamePieceState.IDLE;
    private double m_stateEntryTime = 0;

    // Sensor readings (these would come from actual hardware)
    private boolean m_hasGamePiece = false;
    private boolean m_intakeCurrentSpike = false;
    private boolean m_mechanismsInPosition = false;

    // Player inputs
    private boolean m_requestIntake = false;
    private boolean m_requestScore = false;
    private boolean m_requestEject = false;

    // Timeouts for safety
    private static final double ACQUIRING_TIMEOUT = 2.0; // seconds
    private static final double SCORING_TIMEOUT = 1.0;
    private static final double EJECTING_TIMEOUT = 0.5;

    public GamePieceManager() {
        enterState(GamePieceState.IDLE);
    }

    @Override
    public void periodic() {
        // Update sensor readings (implement these methods based on your hardware)
        updateSensors();

        // Run state machine
        GamePieceState nextState = updateStateMachine();

        if (nextState != m_currentState) {
            enterState(nextState);
        }

        // Update outputs based on current state
        updateOutputs();

        // Telemetry
        updateTelemetry();
    }

    /**
     * Main state machine logic. Returns the next state based on current state, sensors, and inputs.
     */
    private GamePieceState updateStateMachine() {
        double timeInState = Timer.getFPGATimestamp() - m_stateEntryTime;

        switch (m_currentState) {
            case IDLE:
                // Transition: Player requests intake
                if (m_requestIntake) {
                    return GamePieceState.SEARCHING;
                }
                // Transition: Unexpected game piece detected (picked up somehow)
                if (m_hasGamePiece) {
                    return GamePieceState.HOLDING;
                }
                break;

            case SEARCHING:
                // Transition: Game piece detected
                if (m_hasGamePiece || m_intakeCurrentSpike) {
                    return GamePieceState.ACQUIRING;
                }
                // Transition: Player cancels intake
                if (!m_requestIntake) {
                    return GamePieceState.IDLE;
                }
                // Transition: Player wants to eject
                if (m_requestEject) {
                    return GamePieceState.IDLE;
                }
                break;

            case ACQUIRING:
                // Transition: Successfully acquired game piece
                if (m_hasGamePiece && !m_intakeCurrentSpike) {
                    return GamePieceState.HOLDING;
                }
                // Transition: Timeout (something went wrong)
                if (timeInState > ACQUIRING_TIMEOUT) {
                    System.err.println("[GamePiece] Acquiring timeout - returning to IDLE");
                    return GamePieceState.IDLE;
                }
                // Transition: Player cancels
                if (m_requestEject) {
                    return GamePieceState.EJECTING;
                }
                break;

            case HOLDING:
                // Transition: Player requests scoring
                if (m_requestScore) {
                    return GamePieceState.PREPARING_TO_SCORE;
                }
                // Transition: Player requests eject
                if (m_requestEject) {
                    return GamePieceState.EJECTING;
                }
                // Transition: Lost game piece somehow
                if (!m_hasGamePiece) {
                    System.err.println("[GamePiece] Lost game piece unexpectedly!");
                    return GamePieceState.ERROR;
                }
                break;

            case PREPARING_TO_SCORE:
                // Transition: Mechanisms in position
                if (m_mechanismsInPosition) {
                    return GamePieceState.READY_TO_SCORE;
                }
                // Transition: Player cancels
                if (!m_requestScore) {
                    return GamePieceState.HOLDING;
                }
                break;

            case READY_TO_SCORE:
                // Transition: Still holding score button, actually score
                if (m_requestScore) {
                    return GamePieceState.SCORING;
                }
                // Transition: Player released button, go back to holding
                if (!m_requestScore) {
                    return GamePieceState.HOLDING;
                }
                break;

            case SCORING:
                // Transition: Game piece released
                if (!m_hasGamePiece) {
                    return GamePieceState.IDLE;
                }
                // Transition: Timeout
                if (timeInState > SCORING_TIMEOUT) {
                    return GamePieceState.IDLE;
                }
                break;

            case EJECTING:
                // Transition: Game piece ejected
                if (!m_hasGamePiece) {
                    return GamePieceState.IDLE;
                }
                // Transition: Timeout
                if (timeInState > EJECTING_TIMEOUT) {
                    return GamePieceState.IDLE;
                }
                break;

            case ERROR:
                // Transition: Manual reset or sensor readings normalize
                if (!m_hasGamePiece && !m_requestIntake && !m_requestScore) {
                    return GamePieceState.IDLE;
                }
                break;
        }

        return m_currentState; // Stay in current state
    }

    /** Called when entering a new state */
    private void enterState(GamePieceState newState) {
        System.out.println("[GamePiece] State: " + m_currentState + " → " + newState);
        m_currentState = newState;
        m_stateEntryTime = Timer.getFPGATimestamp();

        // State entry actions
        switch (newState) {
            case IDLE:
                // Stop all mechanisms
                break;
            case SEARCHING:
                // Start intake motor
                break;
            case ACQUIRING:
                // Keep intake running at full power
                break;
            case HOLDING:
                // Stop intake, maybe light LED
                break;
            case PREPARING_TO_SCORE:
                // Command arm/elevator to scoring position
                break;
            case SCORING:
                // Reverse intake or open claw
                break;
            case EJECTING:
                // Reverse intake at full power
                break;
            default:
                break;
        }
    }

    /** Update mechanism outputs based on current state */
    private void updateOutputs() {
        // This would control actual motors, solenoids, etc.
        // Example:
        // m_intakeMotor.set(getIntakeSpeed());
        // m_led.set(getLEDColor());
    }

    /** Update sensor readings - implement based on your hardware */
    private void updateSensors() {
        // Example implementations:
        // m_hasGamePiece = m_beamBreak.get();
        // m_intakeCurrentSpike = m_intakeMotor.getOutputCurrent() > 20.0;
        // m_mechanismsInPosition = m_arm.atSetpoint() && m_elevator.atSetpoint();
    }

    /** Telemetry and logging */
    private void updateTelemetry() {
        SmartDashboard.putString("GamePiece/State", m_currentState.getDisplayName());
        SmartDashboard.putBoolean("GamePiece/Has Piece", m_hasGamePiece);
        SmartDashboard.putBoolean(
                "GamePiece/Ready to Score", m_currentState == GamePieceState.READY_TO_SCORE);

        Logger.recordOutput("GamePiece/State", m_currentState.toString());
        Logger.recordOutput("GamePiece/HasPiece", m_hasGamePiece);
        Logger.recordOutput("GamePiece/RequestIntake", m_requestIntake);
        Logger.recordOutput("GamePiece/RequestScore", m_requestScore);
    }

    // ==================== PUBLIC API ====================

    /** Request intake of game piece */
    public void requestIntake(boolean enable) {
        m_requestIntake = enable;
    }

    /** Request scoring */
    public void requestScore(boolean enable) {
        m_requestScore = enable;
    }

    /** Request eject */
    public void requestEject(boolean enable) {
        m_requestEject = enable;
    }

    /** Check if robot has a game piece */
    public boolean hasGamePiece() {
        return m_hasGamePiece;
    }

    /** Check if ready to score */
    public boolean isReadyToScore() {
        return m_currentState == GamePieceState.READY_TO_SCORE;
    }

    /** Get current state */
    public GamePieceState getCurrentState() {
        return m_currentState;
    }

    /** Manual override - force to IDLE (emergency reset) */
    public void forceIdle() {
        enterState(GamePieceState.IDLE);
    }
}
