# Vision Integration Checklist

Use this checklist to ensure proper vision integration with your swerve drivetrain.

## Pre-Deployment Checklist

### Hardware Setup
- [ ] All 3 cameras are physically mounted on robot
- [ ] Camera mounts are secure (no vibration)
- [ ] Cameras have clear view of AprilTags
- [ ] Camera lenses are clean
- [ ] Network cables are properly connected
- [ ] Cameras are powered

### PhotonVision Configuration
- [ ] PhotonVision is installed on coprocessor
- [ ] PhotonVision service is running
- [ ] Camera names match code: `lml3`, `lml2R`, `lml2L`
- [ ] All cameras are detected in PhotonVision web UI
- [ ] Camera calibration completed for all 3 cameras
- [ ] Resolution and FPS settings optimized
- [ ] Exposure settings tuned for your field lighting
- [ ] AprilTag pipeline configured for 2025 Reefscape
- [ ] Target filtering enabled (if available)

### Robot Code
- [ ] `Vision` subsystem instantiated in `RobotContainer`
- [ ] `updateVisionMeasurements()` method exists in `RobotContainer`
- [ ] Vision update called in `Robot.robotPeriodic()`
- [ ] Camera transforms match physical mounting locations
- [ ] Alliance-specific tag IDs are correct

### Network Configuration
- [ ] Static IP assigned to vision coprocessor
- [ ] Robot radio/network switch configured
- [ ] PhotonVision accessible from driver station
- [ ] Network bandwidth sufficient (check other devices)
- [ ] Firewall rules allow PhotonVision traffic

## Build and Deploy

- [ ] Code builds without errors
- [ ] No Java warnings related to vision imports
- [ ] PhotonLib vendor dependency present
- [ ] Code deploys successfully to robot
- [ ] Robot boots without errors
- [ ] Driver station shows robot code running

## Initial Testing

### Camera Connection Test
- [ ] All cameras show as connected in NetworkTables
- [ ] PhotonVision web UI accessible from driver station
- [ ] All cameras show live video feed
- [ ] No camera errors in RoboRIO console

### AprilTag Detection Test
- [ ] Position robot 2m from known AprilTag
- [ ] At least one camera detects the tag
- [ ] Tag detection shown in PhotonVision web UI
- [ ] Tag ID displayed correctly
- [ ] Pose ambiguity < 0.2 for good views
- [ ] Multiple cameras can see same tag simultaneously

### Pose Estimation Test
- [ ] Driver station shows robot pose (X, Y, heading)
- [ ] Pose updates when robot moves
- [ ] Pose corrects when viewing AprilTags
- [ ] Pose doesn't jump erratically
- [ ] Pose matches expected field position (±0.5m)

## Field Testing

### Static Test (Robot Not Moving)
- [ ] Place robot at known position (e.g., 2m, 3m, 180°)
- [ ] Point camera at AprilTag
- [ ] Verify pose matches expected position within 10cm
- [ ] Rotate robot, verify heading updates correctly
- [ ] Pose remains stable (no jitter or jumping)

### Dynamic Test (Robot Moving)
- [ ] Drive robot in straight line
- [ ] Pose tracks movement accurately
- [ ] View AprilTag while driving
- [ ] Pose updates and corrects for drift
- [ ] No significant pose jumps or instability

### Aggressive Driving Test
- [ ] Drive with rapid acceleration (cause wheel slip)
- [ ] Note pose drift due to odometry error
- [ ] Drive to view AprilTag
- [ ] Pose corrects itself automatically
- [ ] Pose converges to correct value within 2 seconds

### Multi-Camera Test
- [ ] Position robot where 2+ cameras see different tags
- [ ] Verify both cameras contribute measurements
- [ ] Check that multi-tag standard deviations are used
- [ ] Pose should be more stable than single camera

### Alliance Test
- [ ] Test with blue alliance selected
- [ ] Verify blue tags (17-22) are tracked
- [ ] Test with red alliance selected
- [ ] Verify red tags (6-11) are tracked
- [ ] Other tags are ignored correctly

## Performance Verification

### Latency
- [ ] Vision update loop runs every 20ms (check logs)
- [ ] Camera latency < 50ms (check PhotonVision)
- [ ] Pose updates feel responsive, not delayed

### CPU Usage
- [ ] RoboRIO CPU usage < 80% during normal operation
- [ ] Vision processing not causing brownouts
- [ ] Code cycle time remains consistent

### Network
- [ ] Network bandwidth usage acceptable
- [ ] No packet loss to cameras
- [ ] Driver station maintains connection
- [ ] Camera feeds don't lag or stutter

## Autonomous Testing

### Path Following
- [ ] Run simple autonomous path (straight line)
- [ ] Robot follows path accurately (±10cm)
- [ ] Path completion improved vs. odometry-only
- [ ] No oscillation or instability

### Tag-Relative Navigation
- [ ] Use `getDesiredPoseLeft()` / `getDesiredPoseRight()`
- [ ] Robot calculates target pose correctly
- [ ] PathPlanner or custom controller drives to pose
- [ ] Final position within 5cm of target

## Troubleshooting Checks (If Issues Occur)

### If cameras not connecting:
- [ ] Check camera power (green LED on camera)
- [ ] Verify network cables seated properly
- [ ] Ping camera IP from driver station
- [ ] Restart PhotonVision service
- [ ] Check RoboRIO network tables for camera data

### If no tags detected:
- [ ] Verify alliance color in driver station
- [ ] Check camera exposure (not too bright/dark)
- [ ] Verify tags in camera field of view
- [ ] Check tag orientation (not upside down)
- [ ] Verify AprilTag layout in `Constants.java`

### If pose jumps/instability:
- [ ] Check camera mounting (ensure no vibration)
- [ ] Increase vision standard deviations
- [ ] Increase ambiguity rejection threshold
- [ ] Verify camera calibration accuracy
- [ ] Check for reflective surfaces causing false detections

### If pose drifts (vision not correcting):
- [ ] Decrease vision standard deviations
- [ ] Check that tags are being detected
- [ ] Verify timestamps are reasonable
- [ ] Check that measurements are being added (log in code)
- [ ] Verify camera transforms are correct

## Optimization (Optional)

- [ ] Tune standard deviations for your environment
- [ ] Adjust ambiguity threshold based on accuracy needs
- [ ] Configure camera scoring factors
- [ ] Implement pose validation (reject large jumps)
- [ ] Add telemetry logging for post-match analysis
- [ ] Optimize camera exposure for field lighting
- [ ] Fine-tune camera mounting angles

## Competition Readiness

- [ ] Vision system tested on practice field
- [ ] Vision system tested on competition field
- [ ] Backup plan if vision fails (disable in code)
- [ ] Drive team trained on vision indicators
- [ ] Vision configuration documented
- [ ] Quick camera replacement procedure established
- [ ] Camera calibration files backed up

## Sign-Off

| Task | Completed By | Date | Notes |
|------|--------------|------|-------|
| Hardware Setup | | | |
| PhotonVision Config | | | |
| Code Integration | | | |
| Initial Testing | | | |
| Field Testing | | | |
| Performance Verification | | | |
| Autonomous Testing | | | |
| Competition Readiness | | | |

---

**Notes:**


