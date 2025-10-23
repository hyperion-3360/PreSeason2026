// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
    /** AprilTag field layout for the current game */
    public static final AprilTagFieldLayout tagLayout =
            AprilTagFields.kDefaultField.loadAprilTagLayoutField();

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
        public static final String LIMELIGHT_NAME = "limelight";

        // Camera mounting (adjust these to match your robot!)
        public static final Transform3d ROBOT_TO_LIMELIGHT =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-2.75), // X: back from center
                                Units.inchesToMeters(0), // Y: centered
                                Units.inchesToMeters(34)), // Z: height off ground
                        new Rotation3d(
                                0, // Roll
                                Units.degreesToRadians(15), // Pitch (angled up)
                                0)); // Yaw

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
        // Default alignment distance
        public static final double DEFAULT_ALIGN_DISTANCE = 1.0; // meters from tag

        // Position and angle tolerances
        public static final double POSITION_TOLERANCE = 0.05; // 5 cm
        public static final double ANGLE_TOLERANCE = Units.degreesToRadians(2.0); // 2 degrees

        // Translation PID (X and Y movement)
        public static final double kP_TRANSLATION = 5.0;
        public static final double kI_TRANSLATION = 0.0;
        public static final double kD_TRANSLATION = 0.25;
        public static final double MAX_VELOCITY_TRANSLATION = 8.0; // m/s
        public static final double MAX_ACCELERATION_TRANSLATION = 10.0; // m/s²

        // Rotation PID (Theta)
        public static final double kP_ROTATION = 7.0;
        public static final double kI_ROTATION = 0.0;
        public static final double kD_ROTATION = 0.3;
        public static final double MAX_VELOCITY_ROTATION = 2.5 * Math.PI; // rad/s
        public static final double MAX_ACCELERATION_ROTATION = 5.0 * Math.PI; // rad/s²
    }
}
