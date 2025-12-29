package frc.robot.subsystems.gamepiece;

/**
 * Defines the states for game piece manipulation. This state machine manages the full lifecycle of
 * acquiring, holding, and scoring game pieces.
 */
public enum GamePieceState {
    /** No game piece, all mechanisms idle */
    IDLE("Idle"),

    /** Intake running, searching for game piece */
    SEARCHING("Searching"),

    /** Game piece detected by sensor, actively acquiring */
    ACQUIRING("Acquiring"),

    /** Game piece secured in robot, intake stopped */
    HOLDING("Holding"),

    /** Moving mechanisms to scoring position */
    PREPARING_TO_SCORE("Preparing"),

    /** In position, ready to release game piece */
    READY_TO_SCORE("Ready"),

    /** Actively scoring (releasing game piece) */
    SCORING("Scoring"),

    /** Ejecting unwanted game piece */
    EJECTING("Ejecting"),

    /** Error state - sensor conflict or unexpected condition */
    ERROR("Error");

    private final String displayName;

    GamePieceState(String displayName) {
        this.displayName = displayName;
    }

    public String getDisplayName() {
        return displayName;
    }

    @Override
    public String toString() {
        return displayName;
    }
}
