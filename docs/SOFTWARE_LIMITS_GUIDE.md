# Software Limits System - Complete Guide

## What Are Software Limits?

Software limits prevent your robot's mechanisms from moving beyond safe positions, protecting hardware from damage. They're like invisible walls that stop motors before they hit physical limits.

### Why You Need Them:

**Without Software Limits:**
- Arm crashes into robot frame → bent shaft, broken gearbox ($500+ damage)
- Elevator extends too far → cable snaps, mechanism falls
- Climber retracts too far → motor stalls, burns out
- Wrist rotates past cable routing → wires get ripped out

**With Software Limits:**
- ✅ Motors stop automatically before damage occurs
- ✅ Warning zone alerts before reaching limits
- ✅ Logged violations for debugging
- ✅ Safety override for emergency situations

---

## Features

### 1. Hard Limits
Position values that **cannot be exceeded**:
```java
// Arm can only move between -5° and 110°
SoftwareLimit armLimit = new SoftwareLimit("Arm", -5.0, 110.0, 10.0, "degrees");

// Trying to go to 120° → automatically clamped to 110°
double safePosition = armLimit.clamp(120.0); // Returns 110.0
```

### 2. Soft Limits (Warning Zone)
Area near limits that triggers warnings:
```java
// Warning zone = 10° from each limit
// Warns when: position < 5° OR position > 100°
armLimit.checkAndWarn(currentPosition);
```

### 3. Velocity Limiting
Prevents movement **toward** limits:
```java
// At 110° (max), trying to move up
double safeVelocity = armLimit.clampVelocity(110.0, +5.0); // Returns 0.0

// At 110°, trying to move down
double safeVelocity = armLimit.clampVelocity(110.0, -5.0); // Returns -5.0 (OK)
```

### 4. Automatic Logging
All limit violations logged to AdvantageKit:
- Current position
- Clamped position
- Whether clamping occurred
- Warning zone status
- Percentage through range

---

## Quick Start - Adding Limits to Your Mechanism

### Step 1: Measure Your Mechanism's Range

**Physical measurement:**
1. Manually move mechanism to minimum position
2. Record encoder value (or reset encoder to 0)
3. Manually move to maximum position
4. Record encoder value

**Example - Arm mechanism:**
```
Minimum: Arm resting on frame = 0 rotations
Maximum: Arm vertical = 0.28 rotations (100:1 gear ratio)
Physical range: 0 to 0.28 rotations = 0° to 100°
```

**Add safety margins:**
```
Software minimum: -5° (allow slight overshoot)
Software maximum: 110° (allow slight overshoot)
```

### Step 2: Add Constants

In `Constants.java`:
```java
public static final class MyMechanismLimits {
    public static final double MIN_POSITION = -5.0;
    public static final double MAX_POSITION = 110.0;
    public static final double WARNING_MARGIN = 10.0; // Warn 10° from limits
}
```

### Step 3: Create SoftwareLimit in Your Subsystem

```java
public class MyArmSubsystem extends SubsystemBase {
    private final TalonFX m_motor;
    private final SoftwareLimit m_limits;

    public MyArmSubsystem() {
        m_motor = new TalonFX(10);

        // Create software limits
        m_limits = new SoftwareLimit(
            "MyArm",
            Constants.MyMechanismLimits.MIN_POSITION,
            Constants.MyMechanismLimits.MAX_POSITION,
            Constants.MyMechanismLimits.WARNING_MARGIN,
            "degrees"
        );
    }

    // ... rest of subsystem
}
```

### Step 4: Use Limits in Your Control Methods

**Position Control:**
```java
public void setPosition(double targetDegrees) {
    // Clamp target to safe range
    double safeTarget = m_limits.clamp(targetDegrees);

    // Convert to motor units and send command
    double motorRotations = degreesToMotorRotations(safeTarget);
    m_motor.setControl(m_positionControl.withPosition(motorRotations));
}
```

**Velocity Control:**
```java
public void setVelocity(double velocityDegreesPerSec) {
    double currentPosition = getPositionDegrees();

    // Stop if at limit and trying to move further
    double safeVelocity = m_limits.clampVelocity(currentPosition, velocityDegreesPerSec);

    m_motor.setControl(m_velocityControl.withVelocity(safeVelocity));
}
```

**Manual Voltage Control:**
```java
public void setVoltage(double voltage) {
    double currentPosition = getPositionDegrees();

    // If at min and trying to go lower, stop
    if (currentPosition <= m_limits.getMinLimit() && voltage < 0) {
        voltage = 0;
    }

    // If at max and trying to go higher, stop
    if (currentPosition >= m_limits.getMaxLimit() && voltage > 0) {
        voltage = 0;
    }

    m_motor.setVoltage(voltage);
}
```

### Step 5: Check Limits in Periodic

```java
@Override
public void periodic() {
    double currentPosition = getPositionDegrees();

    // Check and log warnings
    m_limits.checkAndWarn(currentPosition);

    // AdvantageKit logging
    Logger.recordOutput("MyArm/Position", currentPosition);
    Logger.recordOutput("MyArm/WithinLimits", m_limits.isWithinLimits(currentPosition));
}
```

---

## Example Usage Scenarios

### Scenario 1: Arm Pivot Mechanism

**Hardware:**
- TalonFX motor with 100:1 gear ratio
- Absolute encoder on arm shaft
- Physical range: -10° to 120° (with hard stops)

**Software Configuration:**
```java
// In Constants.java
public static final class ArmLimits {
    public static final double MIN_ANGLE = -5.0;  // 5° margin from hard stop
    public static final double MAX_ANGLE = 115.0; // 5° margin from hard stop
    public static final double WARNING_MARGIN = 10.0;
}

// In ArmSubsystem.java
private final SoftwareLimit m_limits = new SoftwareLimit(
    "Arm",
    ArmLimits.MIN_ANGLE,
    ArmLimits.MAX_ANGLE,
    ArmLimits.WARNING_MARGIN,
    "degrees"
);

public void setAngle(double degrees) {
    double safe = m_limits.clamp(degrees);
    // ... send to motor
}
```

**Result:**
- Driver can command any angle, but mechanism never goes past ±5° from hard stops
- Warning printed when arm approaches 5° or 105°
- All violations logged for post-match analysis

---

### Scenario 2: Linear Elevator

**Hardware:**
- NEO motor with 12:1 gearbox
- 1.5" diameter spool (0.118m circumference)
- Physical travel: 0 to 1.5 meters

**Software Configuration:**
```java
// In Constants.java
public static final class ElevatorLimits {
    public static final double MIN_HEIGHT = 0.02;  // 2cm from bottom
    public static final double MAX_HEIGHT = 1.48;  // 2cm from top
    public static final double WARNING_MARGIN = 0.15; // 15cm warning
}

// In ElevatorSubsystem.java
private final SoftwareLimit m_limits = new SoftwareLimit(
    "Elevator",
    ElevatorLimits.MIN_HEIGHT,
    ElevatorLimits.MAX_HEIGHT,
    ElevatorLimits.WARNING_MARGIN,
    "meters"
);

public void setHeight(double meters) {
    double safe = m_limits.clamp(meters);
    // ... convert to motor rotations and send
}
```

---

### Scenario 3: Turret Rotation

**Hardware:**
- Falcon 500 with 60:1 gear ratio
- 360° continuous rotation with slip ring
- Cable management limits rotation to ±180°

**Software Configuration:**
```java
public static final class TurretLimits {
    public static final double MIN_ANGLE = -175.0;  // 5° margin
    public static final double MAX_ANGLE = 175.0;   // 5° margin
    public static final double WARNING_MARGIN = 20.0;
}

// Turret needs special handling for wraparound
private final SoftwareLimit m_limits = new SoftwareLimit(
    "Turret",
    TurretLimits.MIN_ANGLE,
    TurretLimits.MAX_ANGLE,
    TurretLimits.WARNING_MARGIN,
    "degrees"
);

public void setAngle(double degrees) {
    // Normalize to -180 to +180
    double normalized = ((degrees + 180) % 360) - 180;
    double safe = m_limits.clamp(normalized);
    // ... send to motor
}
```

---

## Advanced Features

### Disable Limits Temporarily (Use with Caution!)

```java
// During match - limits ENABLED
subsystem.setLimitsEnabled(true);

// During calibration/testing - limits DISABLED
subsystem.setLimitsEnabled(false);
subsystem.manualMoveToCalibrationPosition();
subsystem.setLimitsEnabled(true); // Re-enable!
```

**When to disable:**
- ✅ Initial encoder zeroing (with supervision)
- ✅ Finding hard stops during setup
- ❌ NEVER during competition matches
- ❌ NEVER as a "quick fix" for limit issues

### Query Limit Information

```java
SoftwareLimit limits = subsystem.getLimits();

// Get range info
double min = limits.getMinLimit();        // -5.0
double max = limits.getMaxLimit();        // 110.0
double range = limits.getRange();         // 115.0
double center = limits.getCenter();       // 52.5

// Check position status
boolean safe = limits.isWithinLimits(50.0);     // true
boolean warning = limits.isInWarningZone(5.0);  // true
String closest = limits.getClosestLimit(10.0);  // "min"

// Human-readable status
String status = limits.getStatusString(120.0);
// Output: "Arm: 120.00 degrees [VIOLATED]"
```

### Custom Limit Responses

```java
@Override
public void periodic() {
    double pos = getPosition();

    if (!m_limits.isWithinLimits(pos)) {
        // Custom actions on violation

        if (pos < m_limits.getMinLimit()) {
            // Hit minimum - maybe auto-recover?
            setPosition(m_limits.getMinLimit() + 5.0);

            // Flash LEDs red
            ledSubsystem.setColor(Color.RED);

            // Vibrate controller
            Haptics.buzzShort(controller).schedule();
        }
    }
}
```

---

## Debugging Limit Issues

### Problem: Mechanism hits hard stop before software limit

**Cause:** Software limit is too permissive (higher than physical limit)

**Solution:**
1. Measure actual physical limit
2. Reduce `MAX_LIMIT` in Constants.java
3. Test gradually: start conservative, increase slowly

```java
// Before: Software limit at 120°, but arm hits stop at 115°
public static final double MAX_ANGLE = 120.0; // TOO HIGH

// After: Reduce by 5-10° for safety margin
public static final double MAX_ANGLE = 110.0; // SAFE
```

---

### Problem: Software limit stops mechanism too early

**Cause:** Software limit is too restrictive (lower than physical limit)

**Solution:**
1. Verify physical limit with manual movement
2. Increase limit slightly (but maintain safety margin!)
3. Test incrementally

```java
// Before: Software limit at 100°, but arm can safely reach 115°
public static final double MAX_ANGLE = 100.0; // TOO RESTRICTIVE

// After: Increase but keep 5° safety margin
public static final double MAX_ANGLE = 110.0; // (physical 115° - 5° margin)
```

---

### Problem: Limit violations spam console

**Cause:** Normal - happening every loop (50x per second)

**Solution:** Already handled! SoftwareLimit uses cooldown:
- First violation → immediate warning
- Subsequent violations → silent (logged to AdvantageKit)
- Next warning after 2 seconds

To adjust cooldown, edit `WARNING_COOLDOWN` in `SoftwareLimit.java`

---

### Problem: Mechanism "stutters" at limit

**Cause:** Control loop fighting the software limit

**Example:**
```java
// BAD: PID controller trying to reach 120°, but limit is 110°
// Result: Motor keeps trying to reach 120°, limit keeps stopping it
controller.setSetpoint(120.0); // Violates limit!
```

**Solution:** Always clamp setpoints BEFORE sending to controller:
```java
// GOOD: Clamp first, then send safe value
double safeSetpoint = m_limits.clamp(120.0); // Returns 110.0
controller.setSetpoint(safeSetpoint); // Controller reaches 110°, stops cleanly
```

---

## Viewing Limit Data in AdvantageScope

After running robot with limits enabled, open log in AdvantageScope:

### Graph 1: Position vs Limits
Add to line graph:
- `SoftwareLimits/Arm/Position` (current position)
- `SoftwareLimits/Arm/ClampedPosition` (what was actually sent)
- Draw horizontal lines at min/max limits

**What to look for:**
- Gap between Position and ClampedPosition = limit clamping occurred
- Position should never exceed min/max

### Graph 2: Violation Events
Add to line graph:
- `SoftwareLimits/Arm/Violated` (boolean)
- `SoftwareLimits/Arm/InWarningZone` (boolean)

**What to look for:**
- `Violated` spikes = hitting limits
- `InWarningZone` = approaching limits

### Graph 3: Percentage of Range
Add to line graph:
- `SoftwareLimits/Arm/PercentOfRange` (0-100%)

**What to look for:**
- 0% = at minimum
- 100% = at maximum
- 50% = at center
- Useful for seeing mechanism usage patterns

---

## Best Practices

### ✅ DO:
- **Measure physical limits carefully** - use tape measure, protractor, etc.
- **Add safety margins** (5-10° for arms, 2-5cm for linear)
- **Test limits in sim/practice** before competition
- **Log all limit violations** for post-match analysis
- **Use position control with clamped setpoints** for smooth operation
- **Re-check limits after mechanical changes**

### ❌ DON'T:
- **Disable limits during matches** (use them in auto AND teleop!)
- **Set limits at physical hard stops** (always leave margin)
- **Ignore warning zone alerts** (they're telling you something is wrong)
- **Use voltage control without limit checks** (can damage hardware)
- **Copy limits from other robots** (every mechanism is different)
- **Forget to test after changing limits** (test incrementally!)

---

## Testing Checklist

Before competition, verify:

- [ ] Measure physical range of mechanism
- [ ] Set software limits with safety margins
- [ ] Test minimum limit (does it stop safely?)
- [ ] Test maximum limit (does it stop safely?)
- [ ] Test warning zone (does it print warnings?)
- [ ] Test position clamping (command 150°, does it stop at max?)
- [ ] Test velocity limiting (at limit, can it move back but not forward?)
- [ ] Check AdvantageKit logs (are violations being recorded?)
- [ ] Test with real driver inputs (can they break it?)
- [ ] Test in autonomous mode (are limits respected?)

---

## Quick Reference

### Creating a Limit
```java
SoftwareLimit limit = new SoftwareLimit(
    "MechanismName",  // Name for logging
    minPosition,      // Minimum allowed position
    maxPosition,      // Maximum allowed position
    warningMargin,    // Distance from limit to warn
    "units"           // Unit name (degrees, meters, rotations)
);
```

### Using in Position Control
```java
public void setPosition(double target) {
    double safe = m_limits.clamp(target);
    // ... send safe value to motor
}
```

### Using in Velocity Control
```java
public void setVelocity(double velocity) {
    double current = getCurrentPosition();
    double safe = m_limits.clampVelocity(current, velocity);
    // ... send safe velocity to motor
}
```

### Checking in Periodic
```java
@Override
public void periodic() {
    m_limits.checkAndWarn(getCurrentPosition());
}
```

---

## Example: Full Integration

See [ExampleArmSubsystem.java](src/main/java/frc/robot/subsystems/mechanisms/ExampleArmSubsystem.java) for a complete working example with:
- TalonFX motor control
- Motion Magic profiling
- Software limits
- AdvantageKit logging
- Command factories
- Manual control

---

## Summary

Software limits are **critical safety features** that:
- ✅ Prevent expensive hardware damage
- ✅ Allow drivers to command any position safely
- ✅ Provide warnings before hitting limits
- ✅ Log all violations for debugging
- ✅ Are easy to add to any mechanism

**Implementation time:** ~15 minutes per mechanism
**Potential savings:** $500-2000+ in prevented damage

**Every mechanism that moves should have software limits!**
