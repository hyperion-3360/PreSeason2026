# AdvantageKit Logging Setup Guide

### Logging Modes

1. **Real Robot** → Logs to USB stick + publishes to NetworkTables for live viewing
2. **Simulation** → Logs to project directory (`logs/` folder)
3. **Replay** → Loads a previous log file and re-runs it

### What's Being Logged

#### Driver Inputs
- Left joystick (X, Y)
- Right joystick (X rotation)
- Left trigger (L2)
- Right trigger (R2)

#### Robot State
- S-Curve enabled/disabled
- Auto-aim enabled/disabled
- Max speed and angular rate

#### Battery & Brownout
- Battery voltage (real-time)
- Brownout status (GOOD/WARNING/CRITICAL/BROWNOUT)
- Speed scale factor (1.0 = full speed, 0.5 = 50% speed)
- Critical flag (true when battery needs changing)

#### Vision System
- Target detected (true/false)
- Target AprilTag ID
- Target pose on field (X, Y)
- Angle to target (for auto-aim debugging)
- Number of active cameras

#### Swerve Drive
- All standard swerve telemetry (already built into CTRE)
- Module states, velocities, positions
- Odometry updates

---

## How to Use AdvantageKit

### Step 1: Run Your Robot

Just run the robot normally:
- **Simulator**: Use VSCode "Simulate Robot Code" or `./gradlew simulateJava`
- **Real Robot**: Deploy and enable normally

The robot will print to console:
```
[AdvantageKit] Simulation mode - Logging to project directory
[AdvantageKit] Logging initialized successfully
```

### Step 2: Find Your Log Files

**Simulation**: Look in `PreSeason2026/logs/` folder
- Files named like: `Log_2025-10-22_14-30-15.wpilog`

**Real Robot**: Look on USB stick plugged into roboRIO
- Path: `/u/` on roboRIO (mounts as USB drive when connected via USB)

### Step 3: Download AdvantageScope

**Download**: https://github.com/Mechanical-Advantage/AdvantageScope/releases

AdvantageScope is the tool to view/analyze log files. It's like a DVR for your robot.

### Step 4: Open a Log File

1. Launch AdvantageScope
2. Click "File" → "Open"
3. Select your `.wpilog` file
4. The timeline at the bottom shows the entire match/run

### Step 5: Analyze Your Data

#### View Battery Health
1. In left panel, find `Battery/Voltage`
2. Drag it to the line graph area
3. See voltage over time
4. Look for dips below 10.5V (brownout warning)

**Example**:
```
If you see voltage drop to 8V at 2:15 in the match,
that's when brownout protection kicked in!
```

#### Debug Auto-Aim
1. Add these to a graph:
   - `Vision/HasTarget` (true/false)
   - `Vision/AngleToTarget` (radians)
   - `RobotState/AutoAimEnabled` (true/false)
2. Scrub timeline to see when auto-aim was active
3. Check if angle to target was decreasing (robot turning toward target)

**What to look for**:
- Does `HasTarget` stay true during auto-aim?
- Does `AngleToTarget` converge to zero?
- If not, why did vision lose the target?

#### Check Driver Inputs
1. Add `Driver/LeftY`, `Driver/LeftX`, `Driver/RightX`
2. See exactly what the driver was doing
3. Compare input to robot motion (use swerve module outputs)

**Use case**:
- Driver claims "robot wasn't responding" at 1:30
- Log shows `Driver/LeftY = 0.02` (barely moving the stick)
- Exponential scaling made it even smaller
- Robot WAS responding, input was just too small

#### Monitor S-Curve Effects
1. Add `Driver/RightX` (raw input)
2. Add swerve module angular velocity outputs
3. See if S-curve is smoothing the motion
4. Compare with `RobotState/SCurveEnabled = false` periods

---

## Advanced Features

### Replay Mode

You can replay a match and test code changes WITHOUT the physical robot:

1. Copy a log file to: `PreSeason2026/logs/`
2. Run: `./gradlew simulateJava -Preplay`
3. The simulation will replay the exact match
4. Change code (e.g., PID values) and see if it performs better

**Example workflow**:
```bash
# Original match - auto-aim was too slow
./gradlew simulateJava -Preplay  # using original log

# Edit RobotContainer.java - increase AUTO_AIM_kP from 4.0 to 5.0

# Replay again with new PID
./gradlew simulateJava -Preplay  # generates new "_replay" log

# Compare the two logs in AdvantageScope side-by-side
```

### Field2d Visualization

AdvantageScope can show your robot driving on the field:

1. In AdvantageScope, go to "Odometry" tab
2. Add field: `Field2d` or network table path
3. See robot position + AprilTag detections visualized

### 3D Robot Visualization

If you add a robot 3D model, AdvantageScope can show:
- Robot driving on field
- Swerve module angles
- Camera views
- Vision targets

---

## Data You Should Collect

### Pre-Season Testing
- [ ] At least 5 practice runs with different drivers
- [ ] Test auto-aim in various field positions
- [ ] Record battery voltage throughout a full 3-minute run
- [ ] Capture brownout events (intentionally drain battery)

### Competition Day
- [ ] Log EVERY match (auto + teleop)
- [ ] Log EVERY practice match
- [ ] Keep logs organized by event/match number

### Analysis Questions to Answer

**Battery Performance**:
- How low does voltage drop during max acceleration?
- At what match time does brownout warning trigger?
- Do we need better brownout thresholds?

**Auto-Aim Performance**:
- How often does vision lose the target mid-align?
- How long does it take to lock on (average)?
- What's the final alignment error (degrees)?

**Driver Performance**:
- Are drivers using full stick range or staying in deadband?
- Is exponential scaling helping or hurting?
- How often do they accidentally trigger auto-aim (L2+R2)?

**S-Curve Tuning**:
- Is S-curve making driving smoother?
- Is it too aggressive (robot feels sluggish)?
- Do we need different values for X vs Y vs rotation?

---

## Tips for Effective Logging

### During Matches
1. **USB stick must be plugged in before robot boots**
2. Note match number and what happened (write it down)
3. After match, pull USB and back up logs immediately

### Log File Naming
Create a naming convention:
```
Event_MatchType_MatchNumber_Result.wpilog

Examples:
- PreSeason_Practice_001_Success.wpilog
- RegionalQual_Q12_AlignFailed.wpilog
- Championship_SF1_BrownoutAt2min.wpilog
```

### Review Logs as a Team
- After every competition day, review key matches
- Look for patterns across multiple matches
- Make data-driven decisions for next day

---

## What Logs Show That You Can't See Otherwise

| Issue | Without Logs | With Logs |
|-------|-------------|-----------|
| "Robot stopped moving" | Guess (battery? code bug?) | **Proof**: Battery at 7.8V, brownout protection active |
| "Auto-aim isn't working" | Manual testing over and over | **Graph**: Vision lost target after 0.3s, too fast rotation |
| "Drivers say it's too sensitive" | Change values blindly | **Data**: Drivers only use 20% stick range, exponential too strong |
| "Battery died mid-match" | Unknown when it started failing | **Timeline**: Voltage dropped below 10V at 2:15, we have 45 seconds left |

---

## Log File Sizes & Storage

### Expected Sizes
- **3-minute match**: 50-150 MB
- **15-minute practice**: 250-500 MB
- **Full competition day** (20 matches): 1-3 GB
- **Full season** (50+ matches): 5-10 GB

### Storage Tips

**Real Robot (USB Stick)**:
- Use 32GB or 64GB USB stick (holds 100-200 matches)
- After each competition: backup to laptop, clear USB stick
- Format as FAT32 for roboRIO compatibility

**Simulation (Project Directory)**:
- Logs stored in `PreSeason2026/logs/`
- Delete old practice logs regularly (keep only important ones)
- Archive match logs to external drive after season

### Cleaning Up Old Logs

**Windows (PowerShell)**:
```powershell
# Delete sim logs older than 7 days
Get-ChildItem "logs\*.wpilog" | Where-Object {$_.LastWriteTime -lt (Get-Date).AddDays(-7)} | Remove-Item

# Or just delete all sim logs (be careful!)
Remove-Item "logs\*.wpilog"
```

**Keep These Logs**:
- ✅ Competition matches (all of them)
- ✅ Important practice runs where something worked/broke
- ✅ Baseline logs before tuning changes
- ❌ Random sim tests from messing around
- ❌ Failed builds/crashes
- ❌ Practice runs older than 2 weeks

### Reducing Log Size

If storage becomes an issue, you can reduce logged data:

**Option 1: Lower logging frequency** (NOT recommended)
- Change `super(0.02)` to `super(0.05)` in Robot.java
- Reduces from 50Hz to 20Hz
- Makes replay less smooth

**Option 2: Remove less important logs** (Better option)
- Comment out driver trigger axes if not needed
- Remove intermediate calculations
- Keep battery, vision, and swerve data

**Option 3: Selective logging** (Best for competitions)
```java
// Only log detailed data during matches, not practice
if (DriverStation.isFMSAttached()) {
    // Full logging
} else {
    // Minimal logging
}
```

### Backup Strategy

**After Each Event**:
1. Copy USB stick to laptop: `EventName_Date/`
2. Rename logs with match info
3. Clear USB stick for next event
4. Upload important logs to team Google Drive/OneDrive

**Example folder structure**:
```
RobotLogs/
├── 2025-Regional-Week1/
│   ├── Quals/
│   │   ├── Q01_Win.wpilog
│   │   ├── Q05_BrownoutIssue.wpilog
│   └── Playoffs/
│       └── SF1_AutoAimFailed.wpilog
├── 2025-Regional-Week2/
└── 2025-Championships/
```

---

## Next Steps

1. **Build the code** (once Java is configured)
2. **Run simulator** and generate your first log file
3. **Download AdvantageScope** and open the log
4. **Explore the data** - try graphing battery voltage and driver inputs
5. **Test auto-aim** and see if vision tracking looks smooth

---

## Resources

- **AdvantageKit Docs**: https://docs.advantagekit.org/
- **AdvantageScope Download**: https://github.com/Mechanical-Advantage/AdvantageScope/releases
- **Example Team Logs**: Search "FRC 1678 AdvantageKit" to see how top teams use it

---

## Logged Data Reference

Here's the complete list of what's being logged:

### `/Driver/*`
- `Driver/LeftY` - Forward/backward input
- `Driver/LeftX` - Left/right strafe input
- `Driver/RightX` - Rotation input
- `Driver/LeftTrigger` - L2 axis
- `Driver/RightTrigger` - R2 axis

### `/RobotState/*`
- `RobotState/SCurveEnabled` - Motion smoothing active
- `RobotState/AutoAimEnabled` - Auto-rotation to AprilTag active
- `RobotState/MaxSpeed` - Current max speed (m/s)
- `RobotState/MaxAngularRate` - Current max rotation (rad/s)

### `/Battery/*`
- `Battery/Voltage` - Real-time voltage
- `Battery/Status` - GOOD/WARNING/CRITICAL/BROWNOUT
- `Battery/SpeedScale` - Speed multiplier (0.5-1.0)
- `Battery/IsCritical` - Needs immediate battery change

### `/Vision/*`
- `Vision/HasTarget` - AprilTag detected
- `Vision/TargetID` - Which tag (1-16)
- `Vision/TargetPose` - Tag position on field
- `Vision/TargetX` - Tag X coordinate
- `Vision/TargetY` - Tag Y coordinate
- `Vision/AngleToTarget` - Angle to turn for alignment (radians)
- `Vision/CameraCount` - Number of active cameras

### Swerve Drive
- All CTRE Phoenix 6 swerve telemetry (automatic)
- Module positions, velocities, angles
- Odometry pose estimates

---
