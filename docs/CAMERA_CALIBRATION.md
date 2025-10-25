# Camera Position Calibration Guide

## Why Camera Position Matters

PhotonVision uses the camera's 3D position and angle to calculate where AprilTags are on the field. **Incorrect camera measurements = incorrect robot positioning!**

---

## Current Camera Configuration

Located in `Constants.java → VisionConstants`:

```java
public static final Transform3d ROBOT_TO_LIMELIGHT =
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-2.75),  // X: 2.75" behind robot center
            Units.inchesToMeters(0),       // Y: centered (left/right)
            Units.inchesToMeters(34)       // Z: 34" above ground
        ),
        new Rotation3d(
            0,                            // Roll: 0° (camera level)
            Units.degreesToRadians(-15),  // Pitch: -15° angled DOWN
            0                             // Yaw: 0° (facing forward)
        )
    );
```

---

## How to Measure the Camera

### Step 1: Find Robot Center

The "robot center" is typically:
- Center of the drivetrain (between all 4 wheels)
- Mark this point on your robot with tape

### Step 2: Measure X (Forward/Backward)

```
        Front of Robot
             ↑
    [===================]
    |                   |
    |   Camera          |  ← Camera is BEHIND center
    |      ↓            |
    |    [●]            |  ← Robot Center
    |                   |
    [===================]
             ↓
        Back of Robot

X = -2.75"  (negative because camera is BEHIND center)
```

**How to measure:**
1. Find distance from robot center to camera
2. If camera is **forward** of center: **positive** (+)
3. If camera is **behind** center: **negative** (-)

### Step 3: Measure Y (Left/Right)

```
    Left Side (+Y)
         ↑
    [==========]
    |          |
    | Camera   |  ← Camera offset to the left
    |    ↓     |
    |   [●]    |  ← Robot Center
    |          |
    [==========]
         ↓
    Right Side (-Y)
```

**How to measure:**
1. Find distance from robot center to camera (side-to-side)
2. If camera is **left** of center: **positive** (+)
3. If camera is **right** of center: **negative** (-)
4. If camera is **centered**: 0

### Step 4: Measure Z (Height)

```
    Camera
      [●] ← 34" above ground
       |
       |
       | Z height
       |
       |
    ═════════  ← Ground
```

**How to measure:**
1. Measure from **ground** to **camera lens center**
2. Always positive (unless your camera is underground!)

### Step 5: Measure Pitch Angle

```
Camera Angled DOWN (negative pitch):
         Camera
            ╱
    -15°  ╱  ← Looking down at tag
        ╱
      ╱_____ Horizontal

Tag below camera level


Camera Angled UP (positive pitch):
      _____ Horizontal
      ╲
        ╲  +15°  ← Looking up at tag
          ╲
         Camera

Tag above camera level
```

**How to measure:**
1. Use a protractor or angle finder
2. Measure angle from **horizontal**
3. If camera points **down**: **negative** (-)
4. If camera points **up**: **positive** (+)
5. If camera is **level**: 0°

**For FRC (AprilTags are usually above robot):**
- Most cameras point **UP** 10-30° = **positive** pitch
- If tag is on ground and camera looks down = **negative** pitch

---

## Common Mistakes

### ❌ Wrong Pitch Sign

**Problem:** Camera is angled up but pitch is negative (or vice versa)

**Symptom:**
- Vision pose estimates are way off
- Robot drives in wrong direction
- Alignment fails completely

**Fix:**
```java
// Camera looking UP at tags? Use positive
Units.degreesToRadians(+15)  // UP

// Camera looking DOWN at tags? Use negative
Units.degreesToRadians(-15)  // DOWN
```

---

### ❌ Height Measured from Wrong Point

**Problem:** Measured from bumper or floor mat instead of actual ground

**Symptom:**
- Vision measurements have consistent offset
- Robot thinks it's closer/farther than reality

**Fix:**
- Measure from **field carpet surface** to **camera lens center**
- Don't include carpet padding in measurement

---

### ❌ X/Y Measured from Wrong Origin

**Problem:** Measured from bumper edge instead of robot center

**Symptom:**
- Position estimates are shifted forward/backward
- Alignment stops at wrong distance

**Fix:**
- Always measure from **robot center** (drivetrain center)
- Mark center point with tape to avoid confusion

---

## PhotonVision Configuration

Your camera transform must **match** between:

1. **Robot Code** (Constants.java)
2. **PhotonVision Web Interface**

### To Set in PhotonVision:

1. Open PhotonVision: `http://photonvision.local:5800`
2. Go to **Settings** tab
3. Find **Camera Transform** section
4. Enter the **same values** as in Constants.java:
   - X (forward/back)
   - Y (left/right)
   - Z (height)
   - Roll, Pitch, Yaw

**CRITICAL:** Values must match exactly or pose estimates will be wrong!

---

## Testing Your Camera Calibration

### Method 1: Known Position Test

1. Place robot at a **known position** on field
2. Look at an AprilTag with **known ID**
3. Check console output or AdvantageScope:
   ```
   [Vision] Tag ID 7 at (16.18, 0.88)
   ```
4. Compare to **actual tag position** in field layout
5. If off by more than 10cm, recheck calibration

### Method 2: Alignment Distance Test

1. Use auto-align to stop 1.0m from tag
2. **Physically measure** bumper to tag with tape measure
3. Should be exactly 1.0m (±3cm)
4. If wrong:
   - Check `ROBOT_CENTER_TO_FRONT_BUMPER`
   - Check camera transform (especially pitch)

### Method 3: Multiple Tags Test

1. Drive around field viewing different tags
2. All tag positions should be consistent
3. Robot pose estimate should be smooth
4. If tags "jump" or positions are inconsistent:
   - Camera transform is wrong
   - Or camera calibration in PhotonVision is off

---

## Camera Calibration in PhotonVision

Before using vision, you must **calibrate the camera** in PhotonVision:

1. Print calibration pattern (8.5x11" chessboard)
2. Go to PhotonVision → Cameras → Calibration
3. Follow on-screen instructions (hold pattern at various angles)
4. Take 12+ images with low error
5. Save calibration

**Symptoms of uncalibrated camera:**
- Tag detection works but positions are wrong
- High "ambiguity" values
- Inconsistent pose estimates

---

## Your Specific Setup

Based on your description "camera is at slight angle and above the tag":

```
Camera is ABOVE tag on field (common for FRC)
Camera is angled DOWN to see tag
```

**Correct configuration:**
```java
new Rotation3d(
    0,                           // Roll: 0° (camera not tilted sideways)
    Units.degreesToRadians(-15), // Pitch: -15° DOWN to see lower tags
    0                            // Yaw: 0° (camera faces forward)
)
```

**If your camera is looking UP at tags:**
```java
new Rotation3d(
    0,                           // Roll: 0°
    Units.degreesToRadians(+25), // Pitch: +25° UP to see higher tags
    0                            // Yaw: 0°
)
```

---

## Quick Reference

| Camera Position | X Value | Y Value | Z Value | Pitch |
|----------------|---------|---------|---------|-------|
| Forward, centered, 20" high, angled up 20° | `+5.0"` | `0"` | `20"` | `+20°` |
| Behind center, centered, 30" high, level | `-3.0"` | `0"` | `30"` | `0°` |
| Forward, left side, 25" high, angled down 10° | `+4.0"` | `+6.0"` | `25"` | `-10°` |

**Remember:**
- X: + = forward, - = back
- Y: + = left, - = right
- Z: always positive (height above ground)
- Pitch: + = up, - = down

---

## After Changing Camera Transform

1. **Update Constants.java** with new values
2. **Update PhotonVision** web interface (same values)
3. **Restart PhotonVision** (or reboot robot)
4. **Re-enable robot** (vision needs fresh start)
5. **Test alignment** to verify improvement

---

## Still Having Issues?

1. **Check PhotonVision camera calibration** - must be done first
2. **Verify field layout** - correct year loaded
3. **Check alliance color** - set in Driver Station
4. **Look at console output** - does tag position make sense?
5. **Use AdvantageScope** - graph vision pose vs odometry pose

---

**Bottom Line:** Accurate camera position = accurate vision alignment. Measure twice, code once! 📏
