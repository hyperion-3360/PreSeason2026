# Quick Start: Vision Integration

## What Was Changed

Two files were modified to integrate vision with your swerve drivetrain:

### 1. `RobotContainer.java`
- Added `Vision` subsystem instantiation
- Added `updateVisionMeasurements()` method

### 2. `Robot.java`
- Added call to `updateVisionMeasurements()` in `robotPeriodic()`

## What This Does

Your swerve drivetrain now automatically receives pose corrections from your 3 PhotonVision cameras:
- **lml3** (back camera, elevated)
- **lml2R** (front right camera)
- **lml2L** (front left camera)

The system:
1. ✅ Detects AprilTags on the field
2. ✅ Calculates robot pose from each camera
3. ✅ Filters out poor quality detections
4. ✅ Fuses vision data with wheel odometry
5. ✅ Provides accurate pose for autonomous and telemetry

## Before You Deploy

### Verify Camera Names in PhotonVision
Your code expects these exact camera names:
- `lml3`
- `lml2R`
- `lml2L`

If your cameras have different names, update them in `Vision.java` (lines 98-104).

### Verify Camera Mounting
The camera transforms in `Vision.java` should match your physical robot:

**lml3** (back, elevated):
```java
Translation: (12.25", 0", 34") from robot center
Rotation: 0° roll, +15° pitch (tilted up), 0° yaw
```

**lml2R** (front right):
```java
Translation: (12.25", -11.125", 7.25") from robot center
Rotation: 0° roll, -15° pitch (tilted down), +19.7° yaw (angled right)
```

**lml2L** (front left):
```java
Translation: (12.25", +11.125", 7.25") from robot center
Rotation: 0° roll, -15° pitch (tilted down), -19.7° yaw (angled left)
```

⚠️ **Important**: Measure your actual camera positions and update the transforms if they don't match!

## Building and Deploying

1. Open project in VS Code with WPILib
2. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
3. Select "WPILib: Build Robot Code"
4. If successful, select "WPILib: Deploy Robot Code"

## Testing

### Step 1: Check Camera Connections
After deploying, check that cameras are connected:
- Open Shuffleboard or SmartDashboard
- Enable the robot
- Look for PhotonVision network tables

### Step 2: Verify AprilTag Detection
1. Place robot ~2-3 meters from an AprilTag
2. Point a camera at the tag
3. Check PhotonVision web interface (usually http://photonvision.local or http://10.TE.AM.11)
4. Verify tag is being detected with low ambiguity

### Step 3: Test Pose Estimation
1. Enable the robot in a known position (e.g., origin)
2. Check pose on dashboard (X, Y, Rotation)
3. Drive around while viewing AprilTags
4. Pose should remain accurate and correct itself when viewing tags

### Step 4: Test During Match Play
- Drive aggressively (wheel slip)
- Pose should correct itself when you see AprilTags
- Autonomous paths should follow more accurately

## Troubleshooting

| Problem | Solution |
|---------|----------|
| **Cameras not connecting** | Check network, camera names, and PhotonVision service |
| **No tags detected** | Verify alliance color detection, check camera exposure |
| **Pose jumps** | Increase standard deviations in `VisionCamera.java` |
| **Pose drifts** | Decrease standard deviations, check camera calibration |
| **Build fails** | Check imports, verify PhotonLib vendor dependency |

## Key Files

- `Vision.java` - Main vision subsystem, camera configuration
- `VisionCamera.java` - Individual camera wrapper, pose estimation
- `RobotContainer.java` - Integration point, calls vision updates
- `Robot.java` - Triggers vision updates every loop
- `CommandSwerveDrivetrain.java` - Receives vision measurements

## Tuning Parameters

If you need to adjust vision behavior:

**Trust vision more** (for less odometry drift):
```java
// In VisionCamera.java (lines 27-28)
m_singleTagStdDevs = VecBuilder.fill(2, 2, 4);   // Lower = more trust
m_multiTagStdDevs = VecBuilder.fill(1.5, 1.5, 3);
```

**Trust vision less** (for less pose jumping):
```java
// In VisionCamera.java (lines 27-28)
m_singleTagStdDevs = VecBuilder.fill(6, 6, 12);  // Higher = less trust
m_multiTagStdDevs = VecBuilder.fill(5, 5, 10);
```

**Filter more aggressively**:
```java
// In VisionCamera.java (line 121)
if (targetsList.get(index).getPoseAmbiguity() > 0.15) {  // Lower = stricter
```

## Documentation

For more details, see:
- `VISION_INTEGRATION.md` - Complete technical documentation
- `VISION_INTEGRATION_SUMMARY.md` - Detailed changes and recommendations

## Support

If you have questions or issues:
1. Check the documentation files
2. Review PhotonVision logs
3. Check WPILib documentation on pose estimation
4. Verify your camera calibration is accurate

