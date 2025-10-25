package frc.robot.subsystems.util;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

/**
 * Software limit system for mechanisms. Prevents hardware from moving beyond safe positions and
 * provides warnings when approaching limits.
 *
 * <p>Features: - Hard limits (cannot exceed) - Soft limits (warning zone) - Automatic clamping -
 * Logging and alerts - Configurable hysteresis
 */
public class SoftwareLimit {
    private final String m_name;
    private final double m_minLimit;
    private final double m_maxLimit;
    private final double m_warningMargin;
    private final String m_unit;

    private boolean m_lastViolationState = false;
    private double m_lastWarningTime = 0;
    private static final double WARNING_COOLDOWN = 2.0; // seconds between warnings

    /**
     * Creates a new SoftwareLimit.
     *
     * @param name Name of the mechanism (for logging)
     * @param minLimit Minimum allowed position
     * @param maxLimit Maximum allowed position
     * @param warningMargin Distance from limit to start warning (in same units)
     * @param unit Unit of measurement (e.g., "degrees", "meters", "rotations")
     */
    public SoftwareLimit(
            String name, double minLimit, double maxLimit, double warningMargin, String unit) {
        if (minLimit >= maxLimit) {
            throw new IllegalArgumentException(
                    "minLimit must be less than maxLimit: " + minLimit + " >= " + maxLimit);
        }
        if (warningMargin < 0) {
            throw new IllegalArgumentException("warningMargin must be non-negative");
        }

        m_name = name;
        m_minLimit = minLimit;
        m_maxLimit = maxLimit;
        m_warningMargin = warningMargin;
        m_unit = unit;

        System.out.println(
                String.format(
                        "[SoftwareLimit] %s: [%.2f, %.2f] %s (warning margin: %.2f)",
                        name, minLimit, maxLimit, unit, warningMargin));
    }

    /**
     * Checks if a position is within the hard limits.
     *
     * @param position Current position
     * @return true if within limits
     */
    public boolean isWithinLimits(double position) {
        return position >= m_minLimit && position <= m_maxLimit;
    }

    /**
     * Checks if a position is in the warning zone (approaching limits).
     *
     * @param position Current position
     * @return true if in warning zone
     */
    public boolean isInWarningZone(double position) {
        return (position >= m_minLimit && position <= m_minLimit + m_warningMargin)
                || (position <= m_maxLimit && position >= m_maxLimit - m_warningMargin);
    }

    /**
     * Clamps a position to be within the hard limits.
     *
     * @param position Desired position
     * @return Clamped position
     */
    public double clamp(double position) {
        boolean violated = !isWithinLimits(position);

        // Log when limit is first violated
        if (violated && !m_lastViolationState) {
            String violationType = position < m_minLimit ? "MINIMUM" : "MAXIMUM";
            double limit = position < m_minLimit ? m_minLimit : m_maxLimit;

            DriverStation.reportWarning(
                    String.format(
                            "[%s] %s LIMIT VIOLATED: %.2f %s (limit: %.2f %s)",
                            m_name, violationType, position, m_unit, limit, m_unit),
                    false);

            Logger.recordOutput("SoftwareLimits/" + m_name + "/Violated", true);
            Logger.recordOutput("SoftwareLimits/" + m_name + "/ViolationType", violationType);
        }

        // Log when limit violation is resolved
        if (!violated && m_lastViolationState) {
            System.out.println(
                    String.format(
                            "[%s] Limit violation resolved: %.2f %s", m_name, position, m_unit));
            Logger.recordOutput("SoftwareLimits/" + m_name + "/Violated", false);
        }

        m_lastViolationState = violated;

        // Clamp to limits
        double clamped = Math.max(m_minLimit, Math.min(m_maxLimit, position));

        // Log the clamped value
        Logger.recordOutput("SoftwareLimits/" + m_name + "/Position", position);
        Logger.recordOutput("SoftwareLimits/" + m_name + "/ClampedPosition", clamped);
        Logger.recordOutput("SoftwareLimits/" + m_name + "/WasClamped", position != clamped);

        return clamped;
    }

    /**
     * Clamps a velocity based on current position. Prevents movement toward limits.
     *
     * @param position Current position
     * @param velocity Desired velocity
     * @return Safe velocity (may be zero if at limit)
     */
    public double clampVelocity(double position, double velocity) {
        // At or beyond minimum limit, only allow positive velocity
        if (position <= m_minLimit && velocity < 0) {
            Logger.recordOutput("SoftwareLimits/" + m_name + "/VelocityClamped", true);
            Logger.recordOutput("SoftwareLimits/" + m_name + "/ClampReason", "At minimum limit");
            return 0.0;
        }

        // At or beyond maximum limit, only allow negative velocity
        if (position >= m_maxLimit && velocity > 0) {
            Logger.recordOutput("SoftwareLimits/" + m_name + "/VelocityClamped", true);
            Logger.recordOutput("SoftwareLimits/" + m_name + "/ClampReason", "At maximum limit");
            return 0.0;
        }

        Logger.recordOutput("SoftwareLimits/" + m_name + "/VelocityClamped", false);
        return velocity;
    }

    /**
     * Checks position and issues warnings if needed. Should be called periodically.
     *
     * @param position Current position
     */
    public void checkAndWarn(double position) {
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        // Check if we should issue a warning
        if (isInWarningZone(position) && currentTime - m_lastWarningTime > WARNING_COOLDOWN) {
            String direction = position < m_minLimit + m_warningMargin ? "MINIMUM" : "MAXIMUM";

            System.out.println(
                    String.format(
                            "[%s] WARNING: Approaching %s limit - %.2f %s",
                            m_name, direction, position, m_unit));

            Logger.recordOutput("SoftwareLimits/" + m_name + "/InWarningZone", true);
            m_lastWarningTime = currentTime;
        } else if (!isInWarningZone(position)) {
            Logger.recordOutput("SoftwareLimits/" + m_name + "/InWarningZone", false);
        }

        // Always log current position relative to limits
        double percentToMin = (position - m_minLimit) / (m_maxLimit - m_minLimit) * 100;
        Logger.recordOutput("SoftwareLimits/" + m_name + "/PercentOfRange", percentToMin);
    }

    /**
     * Gets the minimum limit.
     *
     * @return Minimum position
     */
    public double getMinLimit() {
        return m_minLimit;
    }

    /**
     * Gets the maximum limit.
     *
     * @return Maximum position
     */
    public double getMaxLimit() {
        return m_maxLimit;
    }

    /**
     * Gets the total range of motion.
     *
     * @return Range (max - min)
     */
    public double getRange() {
        return m_maxLimit - m_minLimit;
    }

    /**
     * Gets the center position of the range.
     *
     * @return Center position
     */
    public double getCenter() {
        return (m_minLimit + m_maxLimit) / 2.0;
    }

    /**
     * Checks if a position is closer to min or max limit.
     *
     * @param position Position to check
     * @return "min", "max", or "center"
     */
    public String getClosestLimit(double position) {
        double distToMin = Math.abs(position - m_minLimit);
        double distToMax = Math.abs(position - m_maxLimit);
        double distToCenter = Math.abs(position - getCenter());

        if (distToCenter < distToMin && distToCenter < distToMax) {
            return "center";
        } else if (distToMin < distToMax) {
            return "min";
        } else {
            return "max";
        }
    }

    /**
     * Formats the current status as a human-readable string.
     *
     * @param position Current position
     * @return Status string
     */
    public String getStatusString(double position) {
        String status = String.format("%s: %.2f %s", m_name, position, m_unit);

        if (!isWithinLimits(position)) {
            status += " [VIOLATED]";
        } else if (isInWarningZone(position)) {
            status += " [WARNING]";
        } else {
            status += " [OK]";
        }

        return status;
    }
}
