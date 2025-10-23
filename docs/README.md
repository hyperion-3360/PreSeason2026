# PreSeason2026 Robot Documentation

This folder contains all documentation for the robot code.

## 📚 Documentation Index

### Setup & Configuration Guides

**[ADVANTAGEKIT_SETUP.md](ADVANTAGEKIT_SETUP.md)** - AdvantageKit Data Logging
- How to use replay logging
- What data is being logged
- How to analyze logs with AdvantageScope
- Log file management and storage tips
- Post-match analysis workflow

**[SOFTWARE_LIMITS_GUIDE.md](SOFTWARE_LIMITS_GUIDE.md)** - Mechanism Safety System
- Preventing hardware damage with software limits
- How to add limits to your mechanisms
- Example configurations for common mechanisms
- Debugging limit violations
- Testing checklist

---

## 🚀 Quick Links

### For New Team Members
1. Start with the robot overview (coming soon)
2. Read [ADVANTAGEKIT_SETUP.md](ADVANTAGEKIT_SETUP.md) to understand data logging
3. Review [SOFTWARE_LIMITS_GUIDE.md](SOFTWARE_LIMITS_GUIDE.md) before working on mechanisms

### For Programmers
- **Adding a new mechanism?** → [SOFTWARE_LIMITS_GUIDE.md](SOFTWARE_LIMITS_GUIDE.md)
- **Debugging a match issue?** → [ADVANTAGEKIT_SETUP.md](ADVANTAGEKIT_SETUP.md)
- **Tuning PID values?** → Use AdvantageKit replay (see setup guide)

### For Drivers
- **Want to see your inputs?** → Ask programmers to open logs in AdvantageScope
- **Robot felt weird?** → Logs can show exactly what happened

---

## 📂 Code Organization

```
src/main/java/frc/robot/
├── Constants.java              # All configuration values
├── Robot.java                  # Main robot class with AdvantageKit
├── RobotContainer.java         # Subsystems and commands
├── subsystems/
│   ├── swerve/                # Swerve drive
│   ├── vision/                # AprilTag vision
│   ├── mechanisms/            # Game-specific mechanisms
│   │   └── ExampleArmSubsystem.java
│   └── util/                  # Utilities
│       ├── SoftwareLimit.java # Safety limits
│       ├── BrownoutProtection.java
│       ├── ExponentialScale.java
│       ├── SCurveLimiter.java
│       └── Haptics.java
└── subsystems/commands/       # Custom commands
```

---

## 🎯 Current Features

### Core Systems ✅
- ✅ Swerve drive (CTRE Phoenix 6)
- ✅ Field-centric control
- ✅ PhotonVision AprilTag tracking
- ✅ AdvantageKit replay logging
- ✅ Software safety limits

### Driver Assists ✅
- ✅ Exponential joystick scaling (precision at low speeds)
- ✅ S-curve motion limiting (smooth acceleration)
- ✅ Auto-aim to AprilTag (L2 + R2 for 3 sec)
- ✅ Field-centric reset (L1 for 3 sec)
- ✅ Battery brownout protection with haptic feedback

### Safety Features ✅
- ✅ Software limits for mechanisms
- ✅ Brownout voltage monitoring
- ✅ Speed scaling based on battery
- ✅ Controller haptic alerts
- ✅ Comprehensive logging

### Missing Features ⚠️
- ⚠️ Autonomous routines (CRITICAL - needed for competition!)
- ⚠️ Game-specific subsystems (intake, shooter, etc.)
- ⚠️ Snap-to-angle (D-pad quick rotation)
- ⚠️ Advanced vision (object detection)

---

## 🔧 Common Tasks

### Tuning PID Values
1. Change values in Constants.java
2. Test with robot (real or sim)
3. Check AdvantageKit logs to see performance
4. Iterate until satisfied

### Adding a New Mechanism
1. Create subsystem in `subsystems/mechanisms/`
2. Add constants in Constants.java → MechanismLimits
3. Create SoftwareLimit in subsystem constructor
4. Add AdvantageKit logging in periodic()
5. Test thoroughly before competition

### Debugging Match Issues
1. Get log file from USB stick or sim directory
2. Open in AdvantageScope
3. Look for:
   - Battery voltage drops
   - Limit violations
   - Vision target loss
   - Driver input anomalies
4. Refer to guide for analysis tips

### Recording a Match
Automatic! If AdvantageKit is enabled:
- **Real robot**: Logs to USB stick automatically
- **Simulation**: Logs to `logs/` folder
- **Naming**: `Log_YYYY-MM-DD_HH-MM-SS.wpilog`

---

## 📖 External Resources

### Official Documentation
- [WPILib Docs](https://docs.wpilib.org/)
- [CTRE Phoenix 6 Docs](https://pro.docs.ctr-electronics.com/)
- [PhotonVision Docs](https://docs.photonvision.org/)
- [AdvantageKit Docs](https://docs.advantagekit.org/)

### Tools
- [AdvantageScope Download](https://github.com/Mechanical-Advantage/AdvantageScope/releases)
- [PathPlanner](https://pathplanner.dev/)
- [PhotonVision](https://photonvision.org/)

### Learning Resources
- [Chief Delphi](https://www.chiefdelphi.com/) - FRC Community Forum
- [FRC Discord](https://discord.gg/frc) - Real-time help

---

## 🏆 Competition Checklist

### Before Each Event
- [ ] All documentation reviewed by drivers
- [ ] AdvantageKit logging tested (USB stick in roboRIO)
- [ ] Software limits verified for all mechanisms
- [ ] Battery brownout thresholds confirmed
- [ ] Auto-aim tested with AprilTags
- [ ] Autonomous routines programmed (when implemented)

### At Competition
- [ ] USB stick with 32GB+ plugged into roboRIO
- [ ] Backup logs after each match
- [ ] Review critical logs between matches
- [ ] Check battery voltage logs for replacement timing

### After Competition
- [ ] Backup all logs to team drive
- [ ] Review any match issues with logs
- [ ] Document lessons learned
- [ ] Update code for next event

---

## 📝 Change Log

**Latest Updates:**
- Added AdvantageKit logging system
- Implemented software limits for mechanisms
- Created comprehensive documentation
- Added battery brownout protection with haptics
- Implemented auto-aim to AprilTag feature

**Next Priorities:**
1. Autonomous routines (CRITICAL)
2. Game-specific mechanisms
3. Snap-to-angle driver assist
4. Advanced vision features

---

## 💡 Contributing

When adding new features:
1. Add configuration to Constants.java
2. Use AdvantageKit logging (Logger.recordOutput)
3. Add software limits if mechanism moves
4. Document in this folder
5. Test thoroughly in simulation first

---

**Last Updated:** October 22, 2025
**Robot Name:** PreSeason2026
**Team:** [Your Team Number]
