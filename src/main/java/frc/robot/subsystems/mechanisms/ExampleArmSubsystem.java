package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.util.SoftwareLimit;
import org.littletonrobotics.junction.Logger;

/**
 * Example arm/pivot mechanism subsystem demonstrating software limit usage.
 *
 * This is a TEMPLATE - customize for your actual mechanism: - Change motor CAN IDs - Adjust gear
 * ratios - Tune PID values - Set correct sensor-to-mechanism conversion
 *
 * Safety features: - Software limits prevent mechanism damage - Automatic position clamping -
 * Velocity limiting at boundaries - Warning zone alerts - AdvantageKit logging
 */
public class ExampleArmSubsystem extends SubsystemBase {
    // Hardware
    private final TalonFX m_motor;

    // Software limits
    private final SoftwareLimit m_limits;

    // Control modes
    private final MotionMagicVoltage m_motionMagicControl = new MotionMagicVoltage(0);
    private final VoltageOut m_voltageControl = new VoltageOut(0);

    // State
    private double m_targetAngle = 0.0; // degrees
    private boolean m_limitsEnabled = true;

    /** Creates a new ExampleArmSubsystem. */
    public ExampleArmSubsystem() {
        // Initialize motor (CHANGE CAN ID FOR YOUR ROBOT!)
        m_motor = new TalonFX(10, "rio"); // CAN ID 10 on "rio" bus

        // Configure motor
        configureTalonFX();

        // Create software limits
        m_limits =
                new SoftwareLimit(
                        "Arm",
                        Constants.MechanismLimits.ArmLimits.MIN_ANGLE,
                        Constants.MechanismLimits.ArmLimits.MAX_ANGLE,
                        Constants.MechanismLimits.ArmLimits.WARNING_MARGIN,
                        "degrees");

        // Set initial position to current encoder position
        m_targetAngle = getAngleDegrees();

        System.out.println("[Arm] Initialized at " + m_targetAngle + " degrees");
    }

    /** Configures the TalonFX motor controller. */
    private void configureTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // EXAMPLE PID VALUES - TUNE FOR YOUR MECHANISM!
        config.Slot0.kP = 50.0; // Proportional gain
        config.Slot0.kI = 0.0; // Integral gain
        config.Slot0.kD = 1.0; // Derivative gain
        config.Slot0.kV = 0.12; // Feedforward velocity
        config.Slot0.kS = 0.25; // Feedforward static friction

        // Motion Magic parameters (smooth profiled motion)
        config.MotionMagic.MotionMagicCruiseVelocity = 80; // rotations/sec
        config.MotionMagic.MotionMagicAcceleration = 160; // rotations/sec²
        config.MotionMagic.MotionMagicJerk = 1600; // rotations/sec³

        // Current limits (prevent brownouts and motor damage)
        config.CurrentLimits.SupplyCurrentLimit = 40.0; // amps
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Set brake mode (holds position when disabled)
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply configuration
        m_motor.getConfigurator().apply(config);

        // Zero the encoder (IMPORTANT: Do this with arm at known position!)
        m_motor.setPosition(0.0);
    }

    @Override
    public void periodic() {
        // Get current angle
        double currentAngle = getAngleDegrees();

        // Check software limits and issue warnings
        if (m_limitsEnabled) {
            m_limits.checkAndWarn(currentAngle);
        }

        // Log to AdvantageKit
        Logger.recordOutput("Arm/CurrentAngle", currentAngle);
        Logger.recordOutput("Arm/TargetAngle", m_targetAngle);
        Logger.recordOutput("Arm/LimitsEnabled", m_limitsEnabled);
        Logger.recordOutput("Arm/MotorCurrent", m_motor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Arm/MotorVoltage", m_motor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Arm/AtTarget", isAtTarget());

        // Log limit status
        Logger.recordOutput("Arm/WithinLimits", m_limits.isWithinLimits(currentAngle));
        Logger.recordOutput("Arm/InWarningZone", m_limits.isInWarningZone(currentAngle));
        Logger.recordOutput("Arm/StatusString", m_limits.getStatusString(currentAngle));
    }

    /**
     * Gets the current arm angle in degrees.
     *
     * @return Arm angle in degrees
     */
    public double getAngleDegrees() {
        // ADJUST THIS CONVERSION FOR YOUR MECHANISM!
        // Example: 100:1 gear ratio
        double motorRotations = m_motor.getPosition().getValueAsDouble();
        double gearRatio = 100.0; // motor rotations per arm rotation
        double armRotations = motorRotations / gearRatio;
        double armDegrees = armRotations * 360.0;

        return armDegrees;
    }

    /**
     * Sets the target angle for the arm.
     *
     * @param angleDegrees Target angle in degrees
     */
    public void setAngle(double angleDegrees) {
        // Apply software limits if enabled
        double safeAngle = m_limitsEnabled ? m_limits.clamp(angleDegrees) : angleDegrees;

        m_targetAngle = safeAngle;

        // Convert degrees to motor rotations
        double gearRatio = 100.0;
        double armRotations = safeAngle / 360.0;
        double motorRotations = armRotations * gearRatio;

        // Use Motion Magic for smooth movement
        m_motor.setControl(m_motionMagicControl.withPosition(motorRotations));

        Logger.recordOutput("Arm/SetpointRequested", angleDegrees);
        Logger.recordOutput("Arm/SetpointApplied", safeAngle);
        Logger.recordOutput("Arm/SetpointClamped", angleDegrees != safeAngle);
    }

    /**
     * Manually controls the arm with a voltage.
     *
     * @param voltage Voltage to apply (-12 to +12)
     */
    public void setVoltage(double voltage) {
        // Check if we're at a limit and trying to move further
        double currentAngle = getAngleDegrees();

        if (m_limitsEnabled) {
            // If at min limit and trying to go down, stop
            if (currentAngle <= m_limits.getMinLimit() && voltage < 0) {
                voltage = 0;
                Logger.recordOutput("Arm/VoltageClamped", true);
                Logger.recordOutput("Arm/ClampReason", "At minimum limit");
            }

            // If at max limit and trying to go up, stop
            if (currentAngle >= m_limits.getMaxLimit() && voltage > 0) {
                voltage = 0;
                Logger.recordOutput("Arm/VoltageClamped", true);
                Logger.recordOutput("Arm/ClampReason", "At maximum limit");
            }
        }

        m_motor.setControl(m_voltageControl.withOutput(voltage));
        Logger.recordOutput("Arm/ManualVoltage", voltage);
    }

    /** Stops the arm motor. */
    public void stop() {
        m_motor.setControl(m_voltageControl.withOutput(0));
        Logger.recordOutput("Arm/Stopped", true);
    }

    /**
     * Checks if the arm is at the target position.
     *
     * @return true if within tolerance
     */
    public boolean isAtTarget() {
        double tolerance = 2.0; // degrees
        return Math.abs(getAngleDegrees() - m_targetAngle) < tolerance;
    }

    /**
     * Enables or disables software limits.
     *
     * @param enabled true to enable limits
     */
    public void setLimitsEnabled(boolean enabled) {
        m_limitsEnabled = enabled;
        System.out.println("[Arm] Software limits " + (enabled ? "ENABLED" : "DISABLED"));
        Logger.recordOutput("Arm/LimitsEnabled", enabled);
    }

    /**
     * Gets the software limit object (for advanced usage).
     *
     * @return SoftwareLimit instance
     */
    public SoftwareLimit getLimits() {
        return m_limits;
    }

    // ========== COMMAND FACTORIES ==========

    /**
     * Command to move arm to a specific angle.
     *
     * @param angleDegrees Target angle
     * @return Command that moves arm and finishes when at target
     */
    public Command setAngleCommand(double angleDegrees) {
        return runOnce(() -> setAngle(angleDegrees))
                .andThen(run(() -> {}))
                .until(this::isAtTarget)
                .withName("ArmToAngle_" + angleDegrees);
    }

    /**
     * Command to move arm to minimum safe position.
     *
     * @return Command
     */
    public Command stowCommand() {
        return setAngleCommand(m_limits.getMinLimit() + 5.0).withName("ArmStow");
    }

    /**
     * Command to move arm to center of range.
     *
     * @return Command
     */
    public Command centerCommand() {
        return setAngleCommand(m_limits.getCenter()).withName("ArmCenter");
    }

    /**
     * Command to move arm to maximum safe position.
     *
     * @return Command
     */
    public Command raiseCommand() {
        return setAngleCommand(m_limits.getMaxLimit() - 5.0).withName("ArmRaise");
    }

    /**
     * Command for manual joystick control.
     *
     * @param speedSupplier Supplier providing speed (-1.0 to 1.0)
     * @return Command that runs until interrupted
     */
    public Command manualControlCommand(java.util.function.DoubleSupplier speedSupplier) {
        return run(() -> {
                    double speed = speedSupplier.getAsDouble();
                    double voltage = speed * 6.0; // Scale to ±6V for safety
                    setVoltage(voltage);
                })
                .finallyDo(this::stop)
                .withName("ArmManualControl");
    }
}
