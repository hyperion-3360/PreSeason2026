# Vision Auto-Align Troubleshooting Guide

## Problem: Auto-Align Works on One Alliance But Not the Other

**Symptom**: Auto-align to AprilTag works correctly in simulation on both alliances, but when testing on the real robot, it only works on red alliance (or only on blue alliance).

### Root Cause

This is almost always a **PhotonVision configuration issue**, specifically the "Field Layout Origin" setting.

### The Issue

FRC fields use different coordinate systems depending on alliance:
- **WPILib (your code)**: Always uses the same coordinate system (0,0 is one corner)
- **PhotonVision**: Needs to be told which alliance you're on to properly transform coordinates

When PhotonVision's origin doesn't match WPILib's, the tag poses it reports are rotated 180°, causing the robot to drive the wrong direction.

---

## How to Fix

### Option 1: Configure PhotonVision Properly (RECOMMENDED)

1. **Connect to PhotonVision Web Interface**
   - Connect to robot WiFi
   - Open browser to `http://photonvision.local:5800`
   - Or use robot IP: `http://10.TE.AM.11:5800` (replace TE.AM with your team number)

2. **Check Field Layout Settings**
   - Go to "Settings" tab
   - Find "AprilTag Field Layout" section
   - **Origin**: Should be set to match WPILib default
   - **For 2024/2025**: Origin should be "Blue Alliance Wall" or "WPILib"

3. **Verify AprilTag Field Layout**
   - Make sure you have the correct year's field layout loaded
   - Default field layout in PhotonVision should match `AprilTagFields.kDefaultField` in your code

4. **Restart PhotonVision**
   - Save settings
   - Reboot PhotonVision or robot
   - Test on both alliances

### Option 2: Test and Debug

When you run auto-align, watch the console output. You should see:

```
[Vision] Alliance: Blue | Aligning to Tag ID 7 at (16.18, 0.88) rotation: 60.0°
[Vision] Target alignment pose: Translation2d(X: 15.18, Y: 0.88), Rotation2d(Rads: -2.09, Deg: -120.00)
```

**What to check:**
1. **Alliance detection**: Is it showing the correct alliance (Red/Blue)?
2. **Tag ID**: Is it targeting the correct tag for your alliance?
3. **Tag position**: Does the X,Y position make sense for that tag on the field?
4. **Tag rotation**: Does the rotation match what you expect?

**Common issues:**
- Tag positions are mirrored (X values are flipped)
- Tag rotations are 180° off
- Wrong tag IDs being selected

---

## Quick Test Procedure

### Test on Both Alliances:

1. **Set Robot to Red Alliance** (FMS or Driver Station)
   - Enable robot
   - Drive in front of a red alliance AprilTag
   - Press R1 (auto-align button)
   - **Expected**: Robot drives toward tag and stops 1m away
   - **Check console**: Note the tag ID, position, and rotation

2. **Set Robot to Blue Alliance**
   - Change alliance in Driver Station
   - Disable/re-enable robot (alliance changes need reboot sometimes)
   - Drive in front of a blue alliance AprilTag
   - Press R1 (auto-align button)
   - **Expected**: Robot drives toward tag and stops 1m away
   - **Check console**: Compare values to red alliance

3. **Compare the Console Output**
   - Are tag rotations consistent?
   - Are tag positions making sense?
   - Is alliance being detected correctly?

---

## PhotonVision Configuration Checklist

- [ ] PhotonVision web interface accessible
- [ ] Correct field layout year loaded (2024/2025)
- [ ] Origin set to "Blue Alliance Wall" or "WPILib Default"
- [ ] Camera calibration completed
- [ ] AprilTag pipeline active and detecting tags
- [ ] Robot restarted after PhotonVision config changes
- [ ] Tested on both red and blue alliances
- [ ] Console output shows correct alliance detection
- [ ] Tag positions match field layout

---

## Advanced: Verify Field Layout in Code

The AprilTag field layout is loaded in Constants.java:

```java
public static final AprilTagFieldLayout tagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
```

This uses WPILib's official field layout for the current year. To verify:

1. Check which tags should be visible from each alliance
2. Red alliance tags: IDs 1, 2, 3, 4 (typically)
3. Blue alliance tags: IDs 5, 6, 7, 8 (typically)
4. Check tag positions in AdvantageScope or WPILib documentation

---

## Still Not Working?

### Check PhotonVision Camera Stream

1. Open PhotonVision web interface
2. Go to "Cameras" tab
3. Verify:
   - Camera is detecting AprilTags (green boxes around tags)
   - Tag IDs are showing correctly
   - Tag pose estimates are updating

### Check Robot Odometry

The auto-align uses the robot's pose estimate from swerve odometry + vision:

1. Open AdvantageScope
2. Load a log file from a test run
3. Check:
   - `Vision/HasTarget` - Was target detected?
   - `Vision/TargetID` - Correct tag?
   - Swerve odometry pose - Is robot localized?

### Check DriverStation Alliance

1. Open FRC Driver Station
2. Go to "Operation" tab
3. Verify alliance color is set correctly
4. **Important**: Alliance color must be set BEFORE enabling robot
5. Changing alliance while enabled may not update PhotonVision

---

## Common Mistakes

❌ **Wrong PhotonVision origin** - Most common issue!
❌ **Old field layout** - Using 2023 layout in 2024 season
❌ **Alliance not set in Driver Station** - Defaults to red
❌ **PhotonVision not restarted** - Changes don't take effect
❌ **Wrong camera transform** - Camera position/rotation in Constants wrong
❌ **Target selection bug** - Code selecting wrong alliance's tags

✅ **Correct PhotonVision origin matching WPILib**
✅ **Current year field layout**
✅ **Alliance set before enabling**
✅ **PhotonVision restarted after config changes**
✅ **Camera transform verified with tape measure**
✅ **Console output checked on both alliances**

---

## Need More Help?

1. **Collect debug data**:
   - Console output from both alliances
   - AdvantageKit log files
   - PhotonVision screenshots showing tag detection

2. **Post on Chief Delphi**:
   - FRC community forum
   - Include: console output, PhotonVision config, robot code version

3. **Check PhotonVision docs**:
   - https://docs.photonvision.org/

---

**Bottom Line**: If it works in sim but not on one alliance IRL, 99% chance it's PhotonVision's origin/field layout configuration. Check PhotonVision settings first!
