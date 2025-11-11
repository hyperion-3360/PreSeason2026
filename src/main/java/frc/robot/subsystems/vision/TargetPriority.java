package frc.robot.subsystems.vision;

/**
 * Defines different strategies for selecting which AprilTag to target when multiple are visible.
 * Each mode uses different weights for distance, quality, and angle.
 */
public enum TargetPriority {
    /** Pure closest - ignore quality */
    CLOSEST("Closest", 1.0, 0.0, 0.0),

    /** Mostly distance, little quality */
    MOSTLY_CLOSEST("Mostly Closest", 1.0, 1.0, 0.0),

    /** Balanced: distance + quality */
    BALANCED("Balanced", 1.0, 3.0, 0.0),

    /** Favor quality over distance */
    MOSTLY_QUALITY("Mostly Quality", 1.0, 8.0, 0.0),

    /** Pure quality - ignore distance */
    BEST_QUALITY("Best Quality", 0.5, 10.0, 0.0),

    /** Prioritize tags in front */
    FRONT_FACING("Front Facing", 0.5, 1.0, 2.0),

    /** Only alliance tags */
    ALLIANCE_ONLY("Alliance Only", 1.0, 0.5, 0.0);

    private final String displayName;
    private final double distanceWeight;
    private final double ambiguityWeight;
    private final double angleWeight;

    TargetPriority(
            String displayName, double distanceWeight, double ambiguityWeight, double angleWeight) {
        this.displayName = displayName;
        this.distanceWeight = distanceWeight;
        this.ambiguityWeight = ambiguityWeight;
        this.angleWeight = angleWeight;
    }

    public String getDisplayName() {
        return displayName;
    }

    public double getDistanceWeight() {
        return distanceWeight;
    }

    public double getAmbiguityWeight() {
        return ambiguityWeight;
    }

    public double getAngleWeight() {
        return angleWeight;
    }

    @Override
    public String toString() {
        return displayName;
    }
}
