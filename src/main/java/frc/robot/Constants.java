// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/** Add your docs here. */
public final class Constants {

    public static class CameraInfo {
        public static final double kCamHeight = 0.0;
        public static final double kCamPitch = 0.0;
    }

    public static final AprilTagFieldLayout tagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
}
