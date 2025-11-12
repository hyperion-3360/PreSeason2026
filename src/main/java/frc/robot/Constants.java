package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /** Robot identification for logging */
    public static final String ROBOT_NAME = "PreSeason2026";

    /** AprilTag field layout for the current game */
    public static final AprilTagFieldLayout tagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    /** Operator Interface (Controller) Constants */
    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final double JOYSTICK_DEADBAND = 0.06;
    }

    /** Drive Constants */
    public static final class DriveConstants {
        // Teleop drive limits
        public static final double MAX_ANGULAR_RATE_TELEOP =
                0.75; // Rotations per second for teleop

        // Exponential scaling for joystick inputs (0 = linear, 1 = fully quadratic)
        // Higher values = more precision at low speeds, typical range: 0.3 to 0.5
        public static final double JOYSTICK_EXPONENTIAL_FACTOR = 0.4;

        // Battery voltage thresholds for brownout protection
        // Typical voltages: Start=12.5V, Mid-match=11.5V, End=10.5-11.5V
        public static final double BATTERY_NOMINAL_VOLTAGE = 12.0; // Fully charged battery voltage
        public static final double BATTERY_WARNING_VOLTAGE =
                8.5; // Start warning (yellow) - battery getting low, plan to swap soon
        public static final double BATTERY_CRITICAL_VOLTAGE =
                7.5; // Critical level (red) - motors may start losing power
        public static final double BATTERY_BROWNOUT_VOLTAGE =
                7.0; // Severe brownout (emergency) - approaching roboRIO brownout (6.8V)

        // Speed scaling factors based on voltage
        // NOTE: Only CRITICAL and BROWNOUT levels limit speed - WARNING just alerts
        public static final double SPEED_SCALE_WARNING = 1.0; // 100% speed - just a warning
        public static final double SPEED_SCALE_CRITICAL =
                0.80; // 80% speed when motors losing power
        public static final double SPEED_SCALE_BROWNOUT = 0.60; // 60% speed at emergency levels

        // S-Curve motion profile limits
        public static final double SCURVE_VX_MAX_VELOCITY = 1.0; // joystick units/s
        public static final double SCURVE_VX_MAX_ACCEL = 6.0; // joystick units/s²
        public static final double SCURVE_VX_MAX_JERK = 120.0; // joystick units/s³

        public static final double SCURVE_VY_MAX_VELOCITY = 1.0; // joystick units/s
        public static final double SCURVE_VY_MAX_ACCEL = 6.0; // joystick units/s²
        public static final double SCURVE_VY_MAX_JERK = 120.0; // joystick units/s³

        public static final double SCURVE_OMEGA_MAX_VELOCITY = 1.0; // joystick units/s
        public static final double SCURVE_OMEGA_MAX_ACCEL = 8.0; // joystick units/s²
        public static final double SCURVE_OMEGA_MAX_JERK = 200.0; // joystick units/s³

        public static final boolean SCURVE_ENABLED_DEFAULT = true;

        // Universal speed limiter (0-100% of max speed)
        // Applies to ALL drive modes: teleop with S-Curve, auto-align, auto-aim, etc.
        // 100 = full speed, 50 = half speed, 0 = no movement
        public static final double SPEED_LIMITER_PERCENT = 100.0; // Percentage of max speed (0-100)
        // Pre-calculated scale factor (performance optimization - avoid division every cycle)
        public static final double SPEED_LIMITER_SCALE = SPEED_LIMITER_PERCENT / 100.0;
    }

    /** Vision Subsystem Constants */
    public static final class VisionConstants {
        // Camera configuration
        public static final String LIMELIGHT_NAME = "lml3";

        // Camera mounting
        // - X: Positive = forward, Negative = backward from robot center
        // - Y: Positive = left, Negative = right from robot center
        // - Z: Height above ground
        // - Pitch: Positive = angled up, Negative = angled down
        public static final Transform3d ROBOT_TO_LIMELIGHT =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-2.75), // X: 2.75" behind center
                                Units.inchesToMeters(0), // Y: centered left/right
                                Units.inchesToMeters(34)), // Z: 34" above ground
                        new Rotation3d(
                                0, // Roll: rotation around forward axis
                                Units.degreesToRadians(-15), // Pitch: -15° angled DOWN (was +15)
                                0)); // Yaw: rotation around vertical axis

        // Target tracking
        public static final double TARGET_LOCK_TIMEOUT = 1.0; // seconds

        // Maximum detection distance for AprilTags
        public static final double MAX_DETECTION_DISTANCE =
                5.0; // meters (ignore tags farther than this)
        public static final double MAX_AUTO_AIM_DISTANCE =
                6.0; // meters (max distance for auto-aim)
        public static final double MAX_AUTO_ALIGN_DISTANCE =
                8.0; // meters (max distance for auto-align)

        // Pose estimation standard deviations (lower = more trust, higher = less trust)
        // RESTORED from AutoAlignWorksGreat branch to fix auto-align precision
        // Higher trust in vision = better pose estimation = accurate auto-align
        public static final double SINGLE_TAG_STD_DEV_X = 4.0;
        public static final double SINGLE_TAG_STD_DEV_Y = 4.0;
        public static final double SINGLE_TAG_STD_DEV_THETA = 8.0;

        public static final double MULTI_TAG_STD_DEV_X = 0.5;
        public static final double MULTI_TAG_STD_DEV_Y = 0.5;
        public static final double MULTI_TAG_STD_DEV_THETA = 1.0;
    }

    /** Auto-Align to AprilTag Constants */
    public static final class AutoAlignConstants {
        /** Motion profile types for alignment */
        public enum MotionProfileType {
            TRAPEZOIDAL("Trapezoidal"),
            EXPONENTIAL("S-Curve");

            private final String displayName;

            MotionProfileType(String displayName) {
                this.displayName = displayName;
            }

            public String getDisplayName() {
                return displayName;
            }
        }

        // Default motion profile type
        public static final MotionProfileType DEFAULT_MOTION_PROFILE =
                MotionProfileType.TRAPEZOIDAL;

        // Robot dimensions
        // RESTORED from AutoAlignWorksGreat: 16.0 inches from center to bumper
        // If this is incorrect for your robot, measure and update accordingly
        public static final double ROBOT_CENTER_TO_FRONT_BUMPER =
                Units.inchesToMeters(16.0); // Distance from robot center to front bumper edge

        // Default alignment distance (from FRONT BUMPER to tag)
        // RESTORED from AutoAlignWorksGreat: 2.2 meters provides better stability
        public static final double DEFAULT_ALIGN_DISTANCE = 2.2; // meters from bumper to tag

        // Position and angle tolerances (realistic for real robot with sensor noise)
        // Too tight = endless oscillation, too loose = inaccurate
        public static final double POSITION_TOLERANCE = 0.03; // 3 cm (was 1cm - too tight)
        public static final double ANGLE_TOLERANCE = Units.degreesToRadians(2.0); // 2° (was 1°)

        // Velocity thresholds for completion (prevents overshoot from inertia)
        public static final double VELOCITY_TOLERANCE = 0.02; // 2 cm/s - must be nearly stopped
        public static final double ANGULAR_VELOCITY_TOLERANCE = 0.05; // 0.05 rad/s (~3 deg/s)

        // Translation PID (X and Y movement) - tuned for precision
        public static final double kP_TRANSLATION = 6.0;
        public static final double kI_TRANSLATION = 0.1;
        public static final double kD_TRANSLATION = 0.4;
        public static final double MAX_VELOCITY_TRANSLATION = 6.0;
        public static final double MAX_ACCELERATION_TRANSLATION = 8.0;

        // Rotation PID (Theta) - tuned for precision
        public static final double kP_ROTATION = 8.0;
        public static final double kI_ROTATION = 0.05;
        public static final double kD_ROTATION = 0.5;
        public static final double MAX_VELOCITY_ROTATION = 2.0 * Math.PI;
        public static final double MAX_ACCELERATION_ROTATION = 4.0 * Math.PI;

        public static final MotionProfileType DEFAULT_PROFILE_TYPE = MotionProfileType.TRAPEZOIDAL;

        // Auto-aim while driving (driver controls translation, robot controls rotation)
        public static final double AUTO_AIM_kP = 4.0;
        public static final double AUTO_AIM_kI = 0.0;
        public static final double AUTO_AIM_kD = 0.15;
        public static final double AUTO_AIM_MAX_ANGULAR_VELOCITY = Math.PI; // rad/s
        public static final double AUTO_AIM_TOLERANCE = Units.degreesToRadians(5.0);

        // Adaptive PID - Gains change based on distance to target
        public static final boolean ENABLE_ADAPTIVE_GAINS = true;

        // Threshold distances for switching gains with hysteresis (meters)
        public static final double ADAPTIVE_THRESHOLD_ENTER_CLOSE = 0.5; // Enter CLOSE mode at 0.5m
        public static final double ADAPTIVE_THRESHOLD_EXIT_CLOSE = 0.6; // Exit CLOSE mode at 0.6m

        // FAR gains (when distance > threshold) - Aggressive for speed
        public static final double kP_TRANSLATION_FAR = 3.0; // Lower P (faster, less precise)
        public static final double kD_TRANSLATION_FAR = 0.15; // Lower D (less damping)
        public static final double kP_ROTATION_FAR = 4.0; // Reduced from 5.0
        public static final double kD_ROTATION_FAR = 0.25; // Reduced from 0.3

        // CLOSE gains (when distance < threshold) - Precise for accuracy
        // Reduced from (8.0, 0.6) to prevent shakiness on real robot
        public static final double kP_TRANSLATION_CLOSE = 4.5; // Was 8.0 - too aggressive
        public static final double kD_TRANSLATION_CLOSE = 0.4; // Was 0.6 - reduced damping
        // Reduced from (10.0, 0.8) to prevent rotation oscillation
        public static final double kP_ROTATION_CLOSE = 5.0; // Was 10.0 - MAJOR shakiness cause
        public static final double kD_ROTATION_CLOSE = 0.5; // Was 0.8 - reduced damping
    }

    /** LED Strip Constants */
    public static class LEDConstants {
        public static final int kLEDPWMPort = 5; // PWM port for LED strip
        public static final int kLEDLength = 30; // Number of LEDs in the strip
    }

    /** Heading Lock Constants */
    public static class HeadingLockConstants {
        // Master enable/disable (toggled by driver with X button hold)
        public static final boolean DEFAULT_ENABLED = false; // Start disabled

        // PID gains for heading correction
        public static final double kP = 5.0; // Proportional gain
        public static final double kI = 0.0; // Integral gain (not needed for heading lock)
        public static final double kD = 0.2; // Derivative gain (damping)

        // Rotation deadband (joystick 0.0-1.0)
        public static final double ROTATION_DEADBAND = 0.05; // 5% stick movement to unlock

        // Correction limits
        public static final double MAX_CORRECTION_RATE =
                0.3; // Max 30% of max angular velocity for corrections
        public static final double LOCK_TOLERANCE_DEGREES = 2.0; // ±2° is acceptable error
    }

    /**
     * Software Limits - Example configurations for common mechanisms
     *
     * <p>These are EXAMPLES - adjust based on your actual robot hardware! Measure your mechanism's
     * physical range and add safety margins.
     */
    public static final class MechanismLimits {
        /** Example: Pivot/Arm limits (in degrees) */
        public static final class ArmLimits {
            public static final double MIN_ANGLE = -5.0;
            public static final double MAX_ANGLE = 110.0;
            public static final double WARNING_MARGIN = 10.0;
        }

        /** Example: Elevator/Linear mechanism limits (in meters) */
        public static final class ElevatorLimits {
            public static final double MIN_HEIGHT = 0.0; // meters - fully retracted
            public static final double MAX_HEIGHT = 1.2; // meters - fully extended
            public static final double WARNING_MARGIN = 0.1; // meters - warn 10cm from limits
        }

        /** Example: Wrist/Intake pivot limits (in degrees) */
        public static final class WristLimits {
            public static final double MIN_ANGLE = -90.0; // degrees - folded back
            public static final double MAX_ANGLE = 90.0; // degrees - folded forward
            public static final double WARNING_MARGIN = 15.0; // degrees
        }

        /** Example: Climber/Winch limits (in rotations) */
        public static final class ClimberLimits {
            public static final double MIN_ROTATIONS = 0.0; // rotations - fully retracted
            public static final double MAX_ROTATIONS = 50.0; // rotations - fully extended
            public static final double WARNING_MARGIN = 5.0; // rotations
        }

        /** Example: Turret rotation limits (in degrees) */
        public static final class TurretLimits {
            public static final double MIN_ANGLE = -180.0; // degrees - full left
            public static final double MAX_ANGLE = 180.0; // degrees - full right
            public static final double WARNING_MARGIN = 20.0; // degrees
        }
    }
}
