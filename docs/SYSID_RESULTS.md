# SysId Characterization Results

## Instructions

Pour chaque routine SysId, note tes résultats ici avant de les copier dans le code.

---

## Translation (Drive Motors)

**Date:**
**Robot Setup:** Robot on ground, ~4 meters clearance
**Test Status:** ❌ Not done

### Results:
```
kS =
kV =
kA =
```

**Where to apply:** `TunerConstants.java` lines 37-44 (driveGains)

---

## Steer (Steering Motors)

**Date:**
**Robot Setup:** Robot on blocks, wheels free to rotate
**Test Status:** ❌ Not done

### Results:
```
kS =
kV =
kA =
kP = (optional)
kD = (optional)
```

**Where to apply:** `TunerConstants.java` lines 26-34 (steerGains)

---

## Rotation (Whole Robot Rotation)

**Date:**
**Robot Setup:** Robot on ground, ~3 meters radius clearance
**Test Status:** ❌ Not done

### Results:
```
kP =
kI =
kD =
```

**Where to apply:** `Constants.java` lines 240-242 (HeadingLockConstants)

---

## Notes

- Translation: Tests forward/backward/sideways movement
- Steer: Tests individual wheel steering response
- Rotation: Tests robot spinning in place
