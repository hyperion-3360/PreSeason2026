# Team Setup Guide

## Prerequisites

- WPILib 2025 installed (includes Java JDK 17)
- Git installed
- VS Code (WPILib version recommended)

## Quick Start (Recommended)

### Option 1: Use WPILib VS Code (Easiest)

1. **Open WPILib VS Code** (not regular VS Code)
   - Windows: Use "WPILib VS Code 2025" shortcut
   - Mac: `/Users/wpilib/2025/vscode/Visual Studio Code.app`
   - Linux: `~/wpilib/2025/vscode/code`

2. **Open Project**
   - File → Open Folder
   - Select `PreSeason2026` directory

3. **Done!** JAVA_HOME is automatically configured

### Option 2: Regular VS Code + Manual JAVA_HOME Setup

If you prefer regular VS Code, you need to set JAVA_HOME permanently:

#### Windows

**PowerShell (Run as Administrator):**
```powershell
[System.Environment]::SetEnvironmentVariable('JAVA_HOME', 'C:\Users\Public\wpilib\2025\jdk', 'User')
```

**Or use GUI:**
1. Press `Windows + R`, type `sysdm.cpl`
2. Advanced → Environment Variables
3. Add User Variable: `JAVA_HOME` = `C:\Users\Public\wpilib\2025\jdk`

#### Mac

Add to `~/.zshrc`:
```bash
export JAVA_HOME="$HOME/wpilib/2025/jdk"
export PATH="$JAVA_HOME/bin:$PATH"
```

Then run: `source ~/.zshrc`

#### Linux

Add to `~/.bashrc`:
```bash
export JAVA_HOME="$HOME/wpilib/2025/jdk"
export PATH="$JAVA_HOME/bin:$PATH"
```

Then run: `source ~/.bashrc`

## Building and Deploying

### Build the Project
```bash
# Windows
.\gradlew build

# Mac/Linux
./gradlew build
```

### Deploy to Robot
```bash
# Windows
.\gradlew deploy

# Mac/Linux
./gradlew deploy
```

### Run Simulation
```bash
# Windows
.\gradlew simulateJava

# Mac/Linux
./gradlew simulateJava
```

## Troubleshooting

### "JAVA_HOME is not set" Error

**Solution 1:** Use WPILib VS Code (recommended)

**Solution 2:** Set JAVA_HOME manually (see above)

**Solution 3:** Verify Java installation
```bash
# Check if Java is installed
java -version

# Should show: openjdk version "17.0.12"
```

### Gradle Commands Not Working

1. Close and reopen your terminal
2. Verify JAVA_HOME is set:
   - Windows: `echo $env:JAVA_HOME`
   - Mac/Linux: `echo $JAVA_HOME`
3. Should show path to WPILib JDK

### VS Code IntelliSense Not Working

1. Open Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P`)
2. Run: "Java: Clean Java Language Server Workspace"
3. Restart VS Code

## Team Resources

- **PhotonVision Cameras**: lml3, lml2R, lml2L
- **Vision Documentation**: See `README_VISION.md`
- **Quick Start**: See `QUICK_START_VISION.md`

## Getting Help

1. Check documentation in project root
2. Ask team programming leads
3. Review Chief Delphi forums
4. Check WPILib documentation: https://docs.wpilib.org/

## Notes for Mixed Environments

- WPILib installs Java in different locations per OS
- Always use WPILib VS Code for consistency
- If using regular VS Code, each team member must set JAVA_HOME once
- JAVA_HOME is machine-specific, not project-specific

