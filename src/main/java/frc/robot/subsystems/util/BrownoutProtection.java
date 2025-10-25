package frc.robot.subsystems.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

/**
 * Monitors battery voltage and provides brownout protection by limiting speed when voltage drops
 * too low. Displays clear warnings when battery needs to be changed. Provides haptic feedback to
 * the driver when battery needs attention.
 */
public class BrownoutProtection {
    public enum BatteryStatus {
        GOOD, // Voltage is nominal (>= warning threshold)
        WARNING, // Voltage is low but acceptable (>= critical threshold)
        CRITICAL, // Voltage is critically low (>= brownout threshold)
        BROWNOUT // Voltage is dangerously low (< brownout threshold)
    }

    private BatteryStatus m_currentStatus = BatteryStatus.GOOD;
    private BatteryStatus m_lastStatus = BatteryStatus.GOOD;
    private double m_lastWarningTime = 0;
    private static final double WARNING_INTERVAL = 5.0; // Warn every 5 seconds
    private final CommandXboxController m_controller;

    // ANSI color codes for console output
    private static final String ANSI_RESET = "\u001B[0m";
    private static final String ANSI_RED = "\u001B[31m";
    private static final String ANSI_YELLOW = "\u001B[33m";
    private static final String ANSI_GREEN = "\u001B[32m";
    private static final String ANSI_BOLD = "\u001B[1m";

    /**
     * Creates a new BrownoutProtection instance.
     *
     * @param controller The driver's controller for haptic feedback
     */
    public BrownoutProtection(CommandXboxController controller) {
        m_controller = controller;
    }

    /**
     * Updates the battery status based on current voltage. Should be called periodically (every
     * robot loop).
     */
    public void update() {
        double voltage = RobotController.getBatteryVoltage();
        m_currentStatus = getBatteryStatus(voltage);

        // Check if status has changed or if it's time for a periodic warning
        boolean statusChanged = m_currentStatus != m_lastStatus;
        boolean timeForWarning = Timer.getFPGATimestamp() - m_lastWarningTime > WARNING_INTERVAL;

        if (statusChanged || (m_currentStatus != BatteryStatus.GOOD && timeForWarning)) {
            logBatteryStatus(voltage);
            m_lastWarningTime = Timer.getFPGATimestamp();
            m_lastStatus = m_currentStatus;

            // Send alert to driver station for critical/brownout conditions
            if (m_currentStatus == BatteryStatus.CRITICAL
                    || m_currentStatus == BatteryStatus.BROWNOUT) {
                DriverStation.reportWarning(
                        String.format(
                                "BATTERY CRITICAL: %.2fV - CHANGE BATTERY IMMEDIATELY!", voltage),
                        false);

                // Trigger haptic feedback - continuous rumble for critical battery
                triggerCriticalHaptics();
            } else if (statusChanged && m_currentStatus == BatteryStatus.GOOD) {
                // Battery recovered - stop rumble
                stopHaptics();
            }
        }

        // Keep rumbling if in critical/brownout state
        if (m_currentStatus == BatteryStatus.CRITICAL
                || m_currentStatus == BatteryStatus.BROWNOUT) {
            maintainCriticalHaptics();
        }
    }

    /** Triggers haptic feedback for critical battery conditions. */
    private void triggerCriticalHaptics() {
        // Start continuous rumble pattern - alternating intensity for attention
        Haptics.rumble(m_controller, 1.0, 1.0, 0.5).schedule();
    }

    /** Maintains haptic feedback during critical battery conditions. */
    private void maintainCriticalHaptics() {
        // Keep a pulsing rumble pattern when battery is critical
        double timeSinceWarning = Timer.getFPGATimestamp() - m_lastWarningTime;
        if (timeSinceWarning < 0.3) {
            // Pulse for first 300ms of each warning interval
            m_controller
                    .getHID()
                    .setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.8);
        } else if (timeSinceWarning < 0.6) {
            // Off for 300ms
            m_controller
                    .getHID()
                    .setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.0);
        } else if (timeSinceWarning < 0.9) {
            // Pulse again
            m_controller
                    .getHID()
                    .setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.8);
        } else {
            // Off until next warning
            m_controller
                    .getHID()
                    .setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.0);
        }
    }

    /** Stops all haptic feedback. */
    private void stopHaptics() {
        m_controller
                .getHID()
                .setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.0);
    }

    /**
     * Gets the current battery status based on voltage.
     *
     * @param voltage Current battery voltage
     * @return Battery status enum
     */
    private BatteryStatus getBatteryStatus(double voltage) {
        if (voltage < Constants.DriveConstants.BATTERY_BROWNOUT_VOLTAGE) {
            return BatteryStatus.BROWNOUT;
        } else if (voltage < Constants.DriveConstants.BATTERY_CRITICAL_VOLTAGE) {
            return BatteryStatus.CRITICAL;
        } else if (voltage < Constants.DriveConstants.BATTERY_WARNING_VOLTAGE) {
            return BatteryStatus.WARNING;
        } else {
            return BatteryStatus.GOOD;
        }
    }

    /**
     * Logs the current battery status with color-coded console output.
     *
     * @param voltage Current battery voltage
     */
    private void logBatteryStatus(double voltage) {
        String statusText;
        String colorCode;

        switch (m_currentStatus) {
            case BROWNOUT:
                colorCode = ANSI_RED + ANSI_BOLD;
                statusText =
                        String.format(
                                "%sSEVERE BROWNOUT: %.2fV - CHANGE BATTERY NOW! Speed limited to 50%%%s",
                                colorCode, voltage, ANSI_RESET);
                // Print multiple times to ensure visibility
                System.err.println("\n" + "=".repeat(80));
                System.err.println(statusText);
                System.err.println("=".repeat(80) + "\n");
                break;

            case CRITICAL:
                colorCode = ANSI_RED;
                statusText =
                        String.format(
                                "%sCRITICAL BATTERY: %.2fV - CHANGE BATTERY IMMEDIATELY! Speed limited to 70%%%s",
                                colorCode, voltage, ANSI_RESET);
                System.err.println(statusText);
                break;

            case WARNING:
                colorCode = ANSI_YELLOW;
                statusText =
                        String.format(
                                "%sBattery Warning: %.2fV - Consider changing battery. Speed limited to 90%%%s",
                                colorCode, voltage, ANSI_RESET);
                System.out.println(statusText);
                break;

            case GOOD:
                colorCode = ANSI_GREEN;
                statusText = String.format("%sBattery OK: %.2fV%s", colorCode, voltage, ANSI_RESET);
                System.out.println(statusText);
                break;
        }
    }

    /**
     * Gets the speed scaling factor based on current battery voltage.
     *
     * @return Speed multiplier (0.0 to 1.0)
     */
    public double getSpeedScaleFactor() {
        switch (m_currentStatus) {
            case BROWNOUT:
                return Constants.DriveConstants.SPEED_SCALE_BROWNOUT;
            case CRITICAL:
                return Constants.DriveConstants.SPEED_SCALE_CRITICAL;
            case WARNING:
                return Constants.DriveConstants.SPEED_SCALE_WARNING;
            case GOOD:
            default:
                return 1.0; // Full speed
        }
    }

    /**
     * Gets the current battery status.
     *
     * @return Current battery status
     */
    public BatteryStatus getStatus() {
        return m_currentStatus;
    }

    /**
     * Checks if battery is in a critical state (critical or brownout).
     *
     * @return true if battery needs immediate attention
     */
    public boolean isCritical() {
        return m_currentStatus == BatteryStatus.CRITICAL
                || m_currentStatus == BatteryStatus.BROWNOUT;
    }

    /**
     * Gets the current battery voltage.
     *
     * @return Battery voltage in volts
     */
    public double getVoltage() {
        return RobotController.getBatteryVoltage();
    }
}
