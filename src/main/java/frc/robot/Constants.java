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
        // NOTE: These are tuned for competition - battery will drop during match!
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
    }

    /** Vision Subsystem Constants */
    public static final class VisionConstants {
        // Camera configuration
        public static final String LIMELIGHT_NAME = "lml3";

        // Camera mounting (adjust these to match your robot!)
        // IMPORTANT: Measure these values carefully!
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

        // Pose estimation standard deviations (lower = more trust)
        public static final double SINGLE_TAG_STD_DEV_X = 4.0;
        public static final double SINGLE_TAG_STD_DEV_Y = 4.0;
        public static final double SINGLE_TAG_STD_DEV_THETA = 8.0;

        public static final double MULTI_TAG_STD_DEV_X = 0.5;
        public static final double MULTI_TAG_STD_DEV_Y = 0.5;
        public static final double MULTI_TAG_STD_DEV_THETA = 1.0;
    }

    /** Auto-Align to AprilTag Constants */
    public static final class AutoAlignConstants {
        // Robot dimensions (adjust to your robot!)
        public static final double ROBOT_CENTER_TO_FRONT_BUMPER =
                Units.inchesToMeters(16.0); // Distance from robot center to front bumper edge

        // Default alignment distance (from FRONT BUMPER to tag)
        public static final double DEFAULT_ALIGN_DISTANCE = 1.0; // meters from bumper to tag

        // Position and angle tolerances (tighter for better precision)
        public static final double POSITION_TOLERANCE = 0.02; // 2 cm
        public static final double ANGLE_TOLERANCE = Units.degreesToRadians(1.0); // 1 degree

        // Translation PID (X and Y movement) - tuned for precision
        public static final double kP_TRANSLATION = 6.0;
        public static final double kI_TRANSLATION =
                0.1; // Small I term to eliminate steady-state error
        public static final double kD_TRANSLATION = 0.4; // Increased from 0.25 for damping
        public static final double MAX_VELOCITY_TRANSLATION =
                6.0; // Reduced from 8.0 for smoother approach
        public static final double MAX_ACCELERATION_TRANSLATION =
                8.0; // Reduced from 10.0 for smoothness

        // Rotation PID (Theta) - tuned for precision
        public static final double kP_ROTATION = 8.0; // Increased from 7.0 for tighter tracking
        public static final double kI_ROTATION = 0.05; // Small I term for final alignment
        public static final double kD_ROTATION = 0.5; // Increased from 0.3 to reduce oscillation
        public static final double MAX_VELOCITY_ROTATION =
                2.0 * Math.PI; // Reduced from 2.5π for smoother rotation
        public static final double MAX_ACCELERATION_ROTATION = 4.0 * Math.PI; // Reduced from 5.0π

        // Auto-aim while driving (driver controls translation, robot controls rotation)
        public static final double AUTO_AIM_kP = 4.0; // Rotation P gain for aim assist
        public static final double AUTO_AIM_kI = 0.0; // Rotation I gain
        public static final double AUTO_AIM_kD = 0.15; // Rotation D gain for damping
        public static final double AUTO_AIM_MAX_ANGULAR_VELOCITY = Math.PI; // rad/s
        public static final double AUTO_AIM_TOLERANCE = Units.degreesToRadians(5.0); // 5 degrees
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
            public static final double MIN_ANGLE = -5.0; // degrees - slightly below horizontal
            public static final double MAX_ANGLE = 110.0; // degrees - straight up
            public static final double WARNING_MARGIN = 10.0; // degrees - warn 10° from limits
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
