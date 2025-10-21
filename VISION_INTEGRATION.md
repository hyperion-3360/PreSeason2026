# Vision Integration with Swerve Drivetrain

## Overview

This document explains how the vision system is integrated with the swerve drivetrain for accurate pose estimation using AprilTag detection via PhotonVision.

## Architecture

### Components

1. **VisionCamera** (`frc.robot.vision.VisionCamera`)
   - Manages individual PhotonVision cameras
   - Handles pose estimation from AprilTag detection
   - Calculates dynamic standard deviations based on detection quality
   - Filters targets by ambiguity (rejects targets with ambiguity > 0.2)

2. **Vision** (`frc.robot.vision.Vision`)
   - Manages multiple cameras (3 cameras: lml3, lml2R, lml2L)
   - Implements target scoring system for tracking best AprilTag
   - Provides pose calculation for reef alignment and navigation
   - Tracks allowed AprilTags based on alliance

3. **CommandSwerveDrivetrain** (`frc.robot.subsystems.swerve.CommandSwerveDrivetrain`)
   - Phoenix 6 swerve drivetrain with integrated pose estimation
   - Uses a Kalman Filter to fuse odometry and vision measurements
   - Provides `addVisionMeasurement()` methods to incorporate vision data

### Camera Configuration

| Camera | Location | Purpose |
|--------|----------|---------|
| **lml3** | Back, elevated (+15° pitch) | Primary - Long-range detection |
| **lml2R** | Front right (-15° pitch, 19.7° yaw) | Secondary - Right-side coverage |
| **lml2L** | Front left (-15° pitch, -19.7° yaw) | Secondary - Left-side coverage |

Camera transforms are defined in `Vision.java` (lines 46-72).

## Integration Flow

### Data Flow Diagram

```
PhotonVision Cameras
        ↓
VisionCamera.updateEstimatedPose()
        ↓
Vision.doPeriodic()
        ↓
RobotContainer.updateVisionMeasurements()
        ↓
CommandSwerveDrivetrain.addVisionMeasurement()
        ↓
SwerveDrivetrain Pose Estimator (Kalman Filter)
        ↓
Updated Robot Pose
```

### Execution Sequence

1. **Robot Periodic** (`Robot.robotPeriodic()` - called every 20ms)
   ```java
   m_robotContainer.updateVisionMeasurements();
   ```

2. **Update Vision Measurements** (`RobotContainer.updateVisionMeasurements()`)
   - Calls `vision.doPeriodic()` to process all cameras
   - Loops through each camera
   - For each active camera with a valid pose estimate:
     - Extracts the estimated pose
     - Retrieves the timestamp
     - Gets dynamic standard deviations
     - Adds measurement to drivetrain

3. **Process Camera Data** (`VisionCamera.updateEstimatedPose()`)
   - Retrieves unread results from PhotonVision
   - Filters targets by ambiguity (> 0.2 rejected)
   - Updates pose estimate using PhotonPoseEstimator
   - Calculates standard deviations dynamically

4. **Vision Tracking** (`Vision.doPeriodic()`)
   - Updates pose estimates for all cameras
   - Applies exponential decay to tag scores
   - Calculates new scores based on:
     - Distance from camera to target
     - Pose ambiguity
     - Camera weighting factors
   - Determines best target (highest score)

## Standard Deviation Calculation

The system uses **adaptive standard deviations** to weight vision measurements based on quality:

### Single Tag Detection
```java
m_singleTagStdDevs = VecBuilder.fill(4, 4, 8);
```
- Higher uncertainty (less trusted)
- Further increased if distance > 4 meters

### Multi-Tag Detection
```java
m_multiTagStdDevs = VecBuilder.fill(3, 3, 6);
```
- Lower uncertainty (more trusted)
- Multiple tags provide better pose constraint

### Distance-Based Adjustment
```java
estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
```
- Standard deviations increase quadratically with distance
- Far targets are less trusted than close targets

### Extreme Case
If only one tag is detected and distance > 4 meters:
```java
estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
```
- Effectively rejects the measurement (infinite uncertainty)

## Key Features

### 1. Ambiguity Filtering
Targets with pose ambiguity > 0.2 are automatically rejected:
```java
if (targetsList.get(index).getPoseAmbiguity() > 0.2) {
    targetsList.remove(index);
}
```

### 2. Target Scoring System
Each allowed AprilTag receives a score based on:
```java
var distanceContribution = Math.exp(-0.5 * distance) * camera.distanceFactor;
var ambiguityContribution = Math.exp(-4 * ambiguity) * camera.ambiguityFactor;
m_AprilTagsScore[tagId] += (distanceContribution * ambiguityContribution);
```

Camera weights:
- **Primary camera (lml3)**: distance factor 2.0, ambiguity factor 2.0
- **Secondary cameras (lml2R/L)**: distance factor 0.5, ambiguity factor 0.5

### 3. Temporal Filtering
Scores decay over time to prevent stale detections from dominating:
```java
m_AprilTagsScore[tag] *= kDecimationFator; // 1.0 / 1.15
```
A score completely vanishes after ~0.5 seconds without detection.

### 4. Alliance-Specific Configuration
The system automatically configures allowed AprilTags based on alliance color:
- **Blue Alliance**: Tags 17-22
- **Red Alliance**: Tags 6-11

## Benefits of This Integration

1. **Improved Pose Accuracy**: Vision measurements correct for wheel slip and odometry drift
2. **Adaptive Weighting**: Quality-based standard deviations ensure poor measurements don't corrupt the pose
3. **Multi-Camera Fusion**: Three cameras provide comprehensive field coverage
4. **Robust Filtering**: Ambiguity and distance filtering prevent false measurements
5. **Seamless Integration**: Works automatically in the background via `robotPeriodic()`

## Troubleshooting

### Vision measurements not updating pose
1. Check camera connections: `camera.isActive()` should return true
2. Verify AprilTag field layout in `Constants.java`
3. Check that PhotonVision is running and configured
4. Ensure camera calibration is accurate

### Pose jumps or instability
1. Reduce vision standard deviations to trust vision less
2. Check for reflective surfaces causing false detections
3. Verify camera mounting is secure (no vibration)
4. Increase ambiguity threshold to reject more uncertain detections

### No AprilTags being tracked
1. Verify alliance color is correctly detected
2. Check that you're viewing allowed tags for your alliance
3. Ensure tags are within camera field of view
4. Verify lighting conditions are adequate

## Configuration Parameters

Key tunable parameters in `Vision.java`:

```java
// Tag scoring factors
kPrimaryTagDistanceFactor = 2.0
kPrimaryTagAmbiguityFactor = 2.0
kSecondaryTagDistanceFactor = 0.5
kSecondaryTagAmbiguityFactor = 0.5

// Score decay
kDecimationFator = 1.0 / 1.15

// Minimum valid score
kMinimumValidScore = 0.01
```

Key tunable parameters in `VisionCamera.java`:

```java
// Standard deviations [x, y, theta]
m_singleTagStdDevs = VecBuilder.fill(4, 4, 8)
m_multiTagStdDevs = VecBuilder.fill(3, 3, 6)

// Ambiguity threshold
getPoseAmbiguity() > 0.2 // rejects target
```

## References

- [WPILib Pose Estimators](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html)
- [PhotonVision Documentation](https://docs.photonvision.org/)
- [CTRE Phoenix 6 Swerve](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/swerve/swerve-overview.html)

