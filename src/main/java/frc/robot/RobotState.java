package frc.robot;

/**
 * Master robot states that coordinate all subsystems. This is the top-level state machine that
 * orchestrates vision, drivetrain, and game piece manipulation.
 */
public enum RobotState {
    /** Manual teleop control - driver has full control */
    TELEOP_MANUAL("Manual Control"),

    /** Searching for game piece - vision looking, auto-align available */
    SEARCHING_FOR_PIECE("Searching"),

    /** Auto-aligning to game piece using vision */
    AUTO_ALIGNING_TO_PIECE("Auto-Align Piece"),

    /** Intaking game piece - vision off, intake running */
    INTAKING_PIECE("Intaking"),

    /** Holding game piece - ready for teleop or scoring */
    HOLDING_PIECE("Holding"),

    /** Auto-aligning to scoring position using vision */
    AUTO_ALIGNING_TO_SCORE("Auto-Align Score"),

    /** Preparing mechanisms for scoring */
    PREPARING_TO_SCORE("Preparing Score"),

    /** Ready to score - waiting for final confirmation */
    READY_TO_SCORE("Ready to Score"),

    /** Actively scoring */
    SCORING("Scoring"),

    /** Emergency stop - all subsystems halt */
    EMERGENCY_STOP("EMERGENCY STOP");

    private final String displayName;

    RobotState(String displayName) {
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
