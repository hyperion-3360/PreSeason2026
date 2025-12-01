package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Auto.pathOnTheFly.EventBuilder.EventMarkerSpecs;
import frc.robot.Auto.pathOnTheFly.PathCommandManager;
import frc.robot.Auto.pathOnTheFly.PathCommandManager.PathGenerationSpecs;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.commands.AlignToTagCommand;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.util.BrownoutProtection;
import frc.robot.subsystems.util.CalibrateAzimuthPersist;
import frc.robot.subsystems.util.Diagnostics;
import frc.robot.subsystems.util.ExponentialScale;
import frc.robot.subsystems.util.Haptics;
import frc.robot.subsystems.util.SCurveLimiter;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.*;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private double MaxSpeed =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =
            RotationsPerSecond.of(Constants.DriveConstants.MAX_ANGULAR_RATE_TELEOP)
                    .in(RadiansPerSecond); // Max angular velocity from Constants

    // Controller - declared early for brownout protection
    private final CommandXboxController joystick =
            new CommandXboxController(Constants.OIConstants.DRIVER_CONTROLLER_PORT);

    // Battery brownout protection with haptic feedback
    private final BrownoutProtection brownoutProtection = new BrownoutProtection(joystick);

    // Exponential scaling for joystick inputs - applied BEFORE S-curve
    private final ExponentialScale expScale =
            new ExponentialScale(Constants.DriveConstants.JOYSTICK_EXPONENTIAL_FACTOR);

    // S-Curve motion limiters - values from Constants.DriveConstants
    private final SCurveLimiter vxLim =
            new SCurveLimiter(
                    Constants.DriveConstants.SCURVE_VX_MAX_VELOCITY,
                    Constants.DriveConstants.SCURVE_VX_MAX_ACCEL,
                    Constants.DriveConstants.SCURVE_VX_MAX_JERK);
    private final SCurveLimiter vyLim =
            new SCurveLimiter(
                    Constants.DriveConstants.SCURVE_VY_MAX_VELOCITY,
                    Constants.DriveConstants.SCURVE_VY_MAX_ACCEL,
                    Constants.DriveConstants.SCURVE_VY_MAX_JERK);
    private final SCurveLimiter omLim =
            new SCurveLimiter(
                    Constants.DriveConstants.SCURVE_OMEGA_MAX_VELOCITY,
                    Constants.DriveConstants.SCURVE_OMEGA_MAX_ACCEL,
                    Constants.DriveConstants.SCURVE_OMEGA_MAX_JERK);

    // Toggle flags
    private boolean sCurveEnabled = Constants.DriveConstants.SCURVE_ENABLED_DEFAULT;
    private boolean autoAimEnabled = false;
    private boolean headingLockEnabled = Constants.HeadingLockConstants.DEFAULT_ENABLED;
    private boolean verboseLoggingEnabled = false; // Toggle for detailed logs (performance)
    private boolean driftEnabled = false;
    private boolean inertiaEnabled = false;

    // Heading Lock state
    private Rotation2d m_lockedHeading = new Rotation2d(); // Heading to maintain
    private boolean m_headingLocked = false; // Is heading currently locked

    // Auto-aim PID controller
    private final PIDController autoAimPID =
            new PIDController(
                    Constants.AutoAlignConstants.AUTO_AIM_kP,
                    Constants.AutoAlignConstants.AUTO_AIM_kI,
                    Constants.AutoAlignConstants.AUTO_AIM_kD);

    // Heading lock PID controller
    private final PIDController headingLockPID =
            new PIDController(
                    Constants.HeadingLockConstants.kP,
                    Constants.HeadingLockConstants.kI,
                    Constants.HeadingLockConstants.kD);

    // Motion profile chooser for alignment
    private final SendableChooser<Constants.AutoAlignConstants.MotionProfileType>
            alignmentProfileChooser = new SendableChooser<>();

    {
        // Configure auto-aim PID
        autoAimPID.enableContinuousInput(-Math.PI, Math.PI);
        autoAimPID.setTolerance(Constants.AutoAlignConstants.AUTO_AIM_TOLERANCE);

        // Configure heading lock PID
        headingLockPID.enableContinuousInput(-Math.PI, Math.PI);
        headingLockPID.setTolerance(
                Math.toRadians(Constants.HeadingLockConstants.LOCK_TOLERANCE_DEGREES));

        // Configure alignment profile chooser
        alignmentProfileChooser.setDefaultOption(
                "Trapezoidal", Constants.AutoAlignConstants.MotionProfileType.TRAPEZOIDAL);
        alignmentProfileChooser.addOption(
                "S-Curve", Constants.AutoAlignConstants.MotionProfileType.EXPONENTIAL);
        SmartDashboard.putData("AutoAlign/Motion Profile", alignmentProfileChooser);
    }

    /**
     * Gets the selected alignment motion profile.
     *
     * @return The selected motion profile type
     */
    public Constants.AutoAlignConstants.MotionProfileType getAlignmentProfile() {
        Constants.AutoAlignConstants.MotionProfileType selected =
                alignmentProfileChooser.getSelected();
        return selected != null ? selected : Constants.AutoAlignConstants.DEFAULT_MOTION_PROFILE;
    }

    // could be used for orbiting around a POI
    private Translation2d m_swerveCenterOfRotation = Translation2d.kZero;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(MaxSpeed * 0.1)
                    .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                    .withCenterOfRotation(m_swerveCenterOfRotation)
                    .withDriveRequestType(
                            DriveRequestType
                                    .OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final VisionSubsystem vision = new VisionSubsystem(drivetrain);
    public final PathCommandManager pathfinding = new PathCommandManager(drivetrain);

    public RobotContainer() {

        configureBindings();

        Diagnostics.bootDiagnostics(
                "FL", RobotConfig.flEnc(), RobotConfig.flSteer(), RobotConfig.flDrive());
        Diagnostics.bootDiagnostics(
                "FR", RobotConfig.frEnc(), RobotConfig.frSteer(), RobotConfig.frDrive());
        Diagnostics.bootDiagnostics(
                "BL", RobotConfig.blEnc(), RobotConfig.blSteer(), RobotConfig.blDrive());
        Diagnostics.bootDiagnostics(
                "BR", RobotConfig.brEnc(), RobotConfig.brSteer(), RobotConfig.brDrive());

        drivetrain.initPathPlanning();

        

    }

    /** Called periodically to update battery voltage monitoring and logging */
    public void periodic() {
        brownoutProtection.update();

        // Always log critical battery info (regardless of verbose toggle)
        Logger.recordOutput("Battery/Voltage", brownoutProtection.getVoltage());
        Logger.recordOutput("Battery/Status", brownoutProtection.getStatus().toString());
        Logger.recordOutput("Battery/SpeedScale", brownoutProtection.getSpeedScaleFactor());
        Logger.recordOutput("Battery/IsCritical", brownoutProtection.isCritical());

        // Verbose logging (toggle to reduce performance impact)
        if (verboseLoggingEnabled) {
            // Log driver inputs (raw, after deadband)
            Logger.recordOutput("Driver/LeftY", joystick.getLeftY());
            Logger.recordOutput("Driver/LeftX", joystick.getLeftX());
            Logger.recordOutput("Driver/RightX", joystick.getRightX());
            Logger.recordOutput("Driver/LeftTrigger", joystick.getLeftTriggerAxis());
            Logger.recordOutput("Driver/RightTrigger", joystick.getRightTriggerAxis());

            // Log control modes
            Logger.recordOutput("RobotState/SCurveEnabled", sCurveEnabled);
            Logger.recordOutput("RobotState/AutoAimEnabled", autoAimEnabled);
            Logger.recordOutput("RobotState/HeadingLockEnabled", headingLockEnabled);

            // Log heading lock state
            Logger.recordOutput("HeadingLock/Locked", m_headingLocked);
            Logger.recordOutput("HeadingLock/LockedHeading", m_lockedHeading.getDegrees());
            var state = drivetrain.getState();
            if (state != null && state.Pose != null) {
                Logger.recordOutput(
                        "HeadingLock/CurrentHeading", state.Pose.getRotation().getDegrees());
                Logger.recordOutput(
                        "HeadingLock/Error",
                        m_lockedHeading.minus(state.Pose.getRotation()).getDegrees());
            }

            // SmartDashboard telemetry
            SmartDashboard.putBoolean("Drive/Heading Lock", headingLockEnabled);
            SmartDashboard.putBoolean("Drive/Heading Locked", m_headingLocked);
            if (m_headingLocked) {
                SmartDashboard.putNumber("Drive/Locked Heading", m_lockedHeading.getDegrees());
            }

            // Log max speeds (useful for seeing brownout effects)
            Logger.recordOutput("RobotState/MaxSpeed", MaxSpeed);
            Logger.recordOutput("RobotState/MaxAngularRate", MaxAngularRate);

            // Log speed limiter status
            Logger.recordOutput(
                    "RobotState/SpeedLimiterPercent",
                    Constants.DriveConstants.SPEED_LIMITER_PERCENT);
            SmartDashboard.putNumber(
                    "Drive/Speed Limiter %", Constants.DriveConstants.SPEED_LIMITER_PERCENT);
        }

        // Always show verbose logging status
        SmartDashboard.putBoolean("Debug/Verbose Logging", verboseLoggingEnabled);

        // Show current SysId routine selection
        SmartDashboard.putString("SysId/Current Routine", drivetrain.getCurrentSysIdRoutineName());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(
                        () -> {
                            double xRaw =
                                    -MathUtil.applyDeadband(
                                            joystick.getLeftY(),
                                            Constants.OIConstants.JOYSTICK_DEADBAND);
                            double yRaw =
                                    -MathUtil.applyDeadband(
                                            joystick.getLeftX(),
                                            Constants.OIConstants.JOYSTICK_DEADBAND);
                            double rRaw =
                                    -MathUtil.applyDeadband(
                                            joystick.getRightX(),
                                            Constants.OIConstants.JOYSTICK_DEADBAND);

                            // Apply exponential scaling for finer control at low speeds
                            double xScaled = expScale.calculate(xRaw);
                            double yScaled = expScale.calculate(yRaw);
                            double rScaled = expScale.calculate(rRaw);

                            double xCmd, yCmd, rCmd;

                            if (sCurveEnabled) {
                                // Apply S-curve motion profiling
                                xCmd = vxLim.calculate(xScaled) * MaxSpeed;
                                yCmd = vyLim.calculate(yScaled) * MaxSpeed;
                                rCmd = omLim.calculate(rScaled) * MaxAngularRate;
                            } else {
                                // Bypass S-curve filters but keep exponential scaling
                                xCmd = xScaled * MaxSpeed;
                                yCmd = yScaled * MaxSpeed;
                                rCmd = rScaled * MaxAngularRate;
                            }

                            // ================================================================
                            // Get drivetrain state once (reused by Heading Lock and Auto-Aim)
                            // ================================================================
                            var drivetrainState = drivetrain.getState();

                            // ================================================================
                            // HEADING LOCK
                            // ================================================================
                            if (headingLockEnabled && !autoAimEnabled) {
                                // Check if rotation stick is near center
                                if (Math.abs(rRaw)
                                        < Constants.HeadingLockConstants.ROTATION_DEADBAND) {
                                    // Rotation stick centered - engage heading lock
                                    if (drivetrainState != null && drivetrainState.Pose != null) {
                                        if (!m_headingLocked) {
                                            // First time entering lock mode - capture current
                                            // heading
                                            m_lockedHeading = drivetrainState.Pose.getRotation();
                                            m_headingLocked = true;
                                            headingLockPID.reset();
                                        }

                                        // Calculate correction to maintain locked heading
                                        Rotation2d currentHeading =
                                                drivetrainState.Pose.getRotation();
                                        double correction =
                                                headingLockPID.calculate(
                                                        currentHeading.getRadians(),
                                                        m_lockedHeading.getRadians());

                                        // Apply correction with rate limiting
                                        // PID output is already in rad/s, don't multiply by
                                        // MaxAngularRate!
                                        rCmd =
                                                MathUtil.clamp(
                                                        correction,
                                                        -MaxAngularRate
                                                                * Constants.HeadingLockConstants
                                                                        .MAX_CORRECTION_RATE,
                                                        MaxAngularRate
                                                                * Constants.HeadingLockConstants
                                                                        .MAX_CORRECTION_RATE);
                                    }
                                } else {
                                    // Rotation stick active - unlock and use manual control
                                    m_headingLocked = false;
                                }
                            }
                            // ================================================================

                            // Auto-aim mode: robot controls rotation, driver controls translation
                            if (autoAimEnabled
                                    && vision.hasTarget()
                                    && vision.isTargetInAutoAimRange()) {
                                // Reuse drivetrainState from above
                                if (drivetrainState != null && drivetrainState.Pose != null) {
                                    Rotation2d currentHeading = drivetrainState.Pose.getRotation();
                                    Rotation2d targetAngle = vision.getAngleToTarget();

                                    rCmd =
                                            MathUtil.clamp(
                                                    autoAimPID.calculate(
                                                            currentHeading.getRadians(),
                                                            targetAngle.getRadians()),
                                                    -Constants.AutoAlignConstants
                                                            .AUTO_AIM_MAX_ANGULAR_VELOCITY,
                                                    Constants.AutoAlignConstants
                                                            .AUTO_AIM_MAX_ANGULAR_VELOCITY);
                                }
                            }

                            // ================================================================

                            // Swerve rotation around a module
                            if (driftEnabled) {
                                double joystickX = joystick.getLeftX();
                                double joystickY = joystick.getLeftY();

                                if (joystickX != 0 && joystickY != 0) {
                                    int robotModulePos[][] = {
                                        // 2 : BackLeft, 3 : BackRight, 0 : FrontLeft,
                                        // 1 : FrontRight
                                        {2, 3},
                                        {0, 1}
                                    };

                                    // depending on controller axis sign gets the necessary module
                                    int frontOrBack = (joystickY > 0) ? 1 : 0;
                                    int leftOrRight = (joystickX > 0) ? 1 : 0;
                                    m_swerveCenterOfRotation =
                                            drivetrain
                                                    .getModuleLocations()[
                                                    robotModulePos[frontOrBack][leftOrRight]];

                                    System.out.println(m_swerveCenterOfRotation);
                                    System.out.println("x" + joystickX);
                                    System.out.println("y" + joystickY);
                                }
                                // It'll basically do nothing since wheel speed is zero
                                else m_swerveCenterOfRotation = Translation2d.kZero;

                                // Transforms the translation into a rotation around an orbit
                                // We can add a value to x and y translation to simulate inertia if
                                // we want
                                if (inertiaEnabled) {}
                                rCmd = xCmd;
                                xCmd = 0.0;
                                yCmd = 0.0;
                                System.out.println(rCmd);
                            } else {
                                m_swerveCenterOfRotation = Translation2d.kZero;
                            }

                            // Apply brownout protection to all commands (translation and rotation)
                            double speedScale = brownoutProtection.getSpeedScaleFactor();
                            xCmd *= speedScale;
                            yCmd *= speedScale;
                            rCmd *= speedScale;

                            // ================================================================
                            // UNIVERSAL SPEED LIMITER (applies to all drive modes)
                            // ================================================================
                            double speedLimiterScale =
                                    Constants.DriveConstants.SPEED_LIMITER_PERCENT / 100.0;
                            xCmd *= speedLimiterScale;
                            yCmd *= speedLimiterScale;
                            rCmd *= speedLimiterScale;

                            // Return normal field-centric drive command
                            return drive.withVelocityX(xCmd)
                                    .withVelocityY(yCmd)
                                    .withRotationalRate(rCmd)
                                    .withCenterOfRotation(m_swerveCenterOfRotation);
                        }));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled()
                .onTrue(
                        Commands.runOnce(
                                        () -> {
                                            vxLim.reset(0);
                                            vyLim.reset(0);
                                            omLim.reset(0);
                                        })
                                .ignoringDisable(true));

        // ========== VISION ALIGNMENT ==========
        // Right bumper: Align to AprilTag at 1 meter distance
        joystick.rightBumper()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    // Reset all driver assist states when starting auto-align
                                    m_headingLocked = false;
                                    headingLockPID.reset();
                                }))
                .whileTrue(
                        AlignToTagCommand.withDefaultDistance(
                                drivetrain, vision, this::getAlignmentProfile))
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    // Reset states again when alignment ends to ensure clean state
                                    m_headingLocked = false;
                                    headingLockPID.reset();

                                    System.out.println(
                                            "[Align] Button released - driver assists reset");
                                },
                                drivetrain));

        // ========== SWERVE WHEEL LOCK ========== \\
        joystick.leftStick()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    driftEnabled = true;
                                    System.out.println("[Drifting command] ENABLED");
                                },
                                drivetrain))
                .onFalse(
                        Commands.runOnce(
                                () -> {
                                    driftEnabled = false;
                                    System.out.println("[Drifting command] DISABLED");
                                },
                                drivetrain));

        PathGenerationSpecs test =
                new PathGenerationSpecs(
                        drivetrain.getState().Pose,
                        List.of(
                                new Pose2d(3, 4, Rotation2d.k180deg),
                                new Pose2d(4, 4, Rotation2d.k180deg)),
                        0.0,
                        0.0,
                        drivetrain.getState().Pose.getRotation().getDegrees(),
                        180.0);
        joystick.y().onTrue(pathfinding.followGeneratedPath(test));

        // ========== AUTO-AIM TOGGLE ==========
        // Hold both triggers (L2 + R2) for 3 seconds to toggle auto-aim mode
        var bothTriggersPressed = joystick.leftTrigger().and(joystick.rightTrigger());

        // Toggle after 3 second hold
        bothTriggersPressed
                .debounce(3.0)
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    autoAimEnabled = !autoAimEnabled;
                                    if (autoAimEnabled) {
                                        autoAimPID.reset();
                                        vision.lockTarget();

                                        // Reset driver assists when enabling auto-aim
                                        m_headingLocked = false;
                                        headingLockPID.reset();

                                        System.out.println(
                                                "[Auto-Aim] ENABLED - Robot will auto-rotate to closest AprilTag");
                                        Haptics.buzzOK(joystick).schedule();
                                    } else {
                                        autoAimPID.reset();
                                        vision.unlockTarget();

                                        // Reset driver assists when disabling auto-aim
                                        m_headingLocked = false;
                                        headingLockPID.reset();

                                        System.out.println(
                                                "[Auto-Aim] DISABLED - Manual rotation control");
                                        Haptics.buzzShort(joystick).schedule();
                                    }
                                }));

        // ========== S-CURVE TOGGLE ==========
        joystick.a()
                .onTrue(
                        Commands.runOnce(
                                        () -> {
                                            boolean newState = !sCurveEnabled;

                                            // If enabling, seed filters to current input so output
                                            // stays continuous
                                            if (newState) {
                                                double xRaw =
                                                        -MathUtil.applyDeadband(
                                                                joystick.getLeftY(),
                                                                Constants.OIConstants
                                                                        .JOYSTICK_DEADBAND);
                                                double yRaw =
                                                        -MathUtil.applyDeadband(
                                                                joystick.getLeftX(),
                                                                Constants.OIConstants
                                                                        .JOYSTICK_DEADBAND);
                                                double rRaw =
                                                        -MathUtil.applyDeadband(
                                                                joystick.getRightX(),
                                                                Constants.OIConstants
                                                                        .JOYSTICK_DEADBAND);
                                                // Apply exponential scaling before seeding filters
                                                vxLim.reset(expScale.calculate(xRaw));
                                                vyLim.reset(expScale.calculate(yRaw));
                                                omLim.reset(expScale.calculate(rRaw));
                                            }

                                            sCurveEnabled = newState;
                                            System.out.println(
                                                    "[Drive] S-curve limiter: "
                                                            + (sCurveEnabled ? "ON" : "OFF"));
                                        })
                                .ignoringDisable(true) // let you toggle while Disabled
                        );

        // ========== HEADING LOCK TOGGLE ==========
        // Hold X button for 3 seconds to toggle heading lock
        joystick.x()
                .debounce(3.0)
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    headingLockEnabled = !headingLockEnabled;
                                    if (headingLockEnabled) {
                                        // Reset heading lock state when enabling
                                        m_lockedHeading = new Rotation2d();
                                        m_headingLocked = false;
                                        headingLockPID.reset();

                                        System.out.println(
                                                "[HeadingLock] ENABLED - Robot will maintain heading when rotation stick centered");
                                        Haptics.buzzOK(joystick).schedule();
                                    } else {
                                        // Unlock heading when disabling
                                        m_headingLocked = false;

                                        System.out.println(
                                                "[HeadingLock] DISABLED - Manual rotation control");
                                        Haptics.buzzShort(joystick).schedule();
                                    }
                                }));

        // ========== VERBOSE LOGGING TOGGLE ==========
        // POV Down to toggle verbose logging (performance optimization)
        joystick.povDown()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    verboseLoggingEnabled = !verboseLoggingEnabled;
                                    if (verboseLoggingEnabled) {
                                        System.out.println(
                                                "[Debug] VERBOSE LOGGING ENABLED - Full telemetry active");
                                        Haptics.buzzOK(joystick).schedule();
                                    } else {
                                        System.out.println(
                                                "[Debug] VERBOSE LOGGING DISABLED - Reduced telemetry for performance");
                                        Haptics.buzzShort(joystick).schedule();
                                    }
                                }));

        // ========== SYSID ROUTINE SELECTOR ==========
        // POV Left/Right to cycle between SysId routines
        joystick.povLeft()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    String routine = drivetrain.cycleSysIdRoutine();
                                    System.out.println("[SysId] Selected routine: " + routine);
                                    Haptics.buzzShort(joystick).schedule();
                                }));

        joystick.povRight()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    String routine = drivetrain.cycleSysIdRoutine();
                                    System.out.println("[SysId] Selected routine: " + routine);
                                    Haptics.buzzShort(joystick).schedule();
                                }));

        // ========== SYSID AUTOMATIC SEQUENCE ==========
        // Hold POV Up for 3 seconds to run complete SysId characterization
        // Runs all 4 tests automatically: quasistatic forward/reverse, dynamic forward/reverse
        // Use POV Left/Right to select routine type (Translation/Steer/Rotation)
        joystick.povUp()
                .debounce(3.0)
                .onTrue(
                        Commands.sequence(
                                        Commands.runOnce(
                                                () ->
                                                        System.out.println(
                                                                "[SysId] Starting "
                                                                        + drivetrain
                                                                                .getCurrentSysIdRoutineName()
                                                                        + " characterization sequence...")),
                                        Haptics.buzzOK(joystick),
                                        Commands.waitSeconds(2.0), // Wait before starting

                                        // Test 1: Quasistatic Forward
                                        Commands.print("[SysId] Test 1/4: Quasistatic Forward"),
                                        Haptics.buzzShort(joystick),
                                        drivetrain.sysIdQuasistatic(Direction.kForward),
                                        Commands.waitSeconds(1.0),

                                        // Test 2: Quasistatic Reverse
                                        Commands.print("[SysId] Test 2/4: Quasistatic Reverse"),
                                        Haptics.buzzShort(joystick),
                                        drivetrain.sysIdQuasistatic(Direction.kReverse),
                                        Commands.waitSeconds(1.0),

                                        // Test 3: Dynamic Forward
                                        Commands.print("[SysId] Test 3/4: Dynamic Forward"),
                                        Haptics.buzzShort(joystick),
                                        drivetrain.sysIdDynamic(Direction.kForward),
                                        Commands.waitSeconds(1.0),

                                        // Test 4: Dynamic Reverse
                                        Commands.print("[SysId] Test 4/4: Dynamic Reverse"),
                                        Haptics.buzzShort(joystick),
                                        drivetrain.sysIdDynamic(Direction.kReverse),
                                        Commands.waitSeconds(1.0),

                                        // Complete
                                        Commands.print(
                                                "[SysId] ✓ All characterization tests complete!"),
                                        Haptics.buzzOK(joystick))
                                .withTimeout(120.0) // 2 minute timeout for safety
                                .finallyDo(
                                        () ->
                                                System.out.println(
                                                        "[SysId] Sequence ended - check logs for data")));

        // ========== FIELD-CENTRIC RESET ==========
        // Press L1 (left bumper) twice quickly to reset field-centric heading
        // (Double-tap prevents accidental resets)
        joystick.leftBumper()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    // prevents vision from "correcting" robot pose when reset
                                    vision.disableVisionUpdates();

                                    drivetrain.seedFieldCentric();
                                    System.out.println(
                                            "[Field-Centric] RESET - Robot forward is now field forward");
                                    Haptics.buzzOK(joystick).schedule();
                                    vision.enableVisionUpdates();
                                }));

        drivetrain.registerTelemetry(logger::telemeterize);

        // ========== CALIBRATION SEQUENCE ==========
        // Combo: hold X + Y + A + B for 3 seconds
        var zeroHold =
                joystick.x().and(joystick.y()).and(joystick.a()).and(joystick.b()).debounce(3.0);
        zeroHold.onTrue(Haptics.buzzOK(joystick).ignoringDisable(true));

        // Sequences (Disabled only)
        var seqRealDisabled =
                Commands.print("[ZeroMode] Starting (DISABLED)…")
                        .andThen(new CalibrateAzimuthPersist().ignoringDisable(true))
                        .andThen(
                                Commands.print(
                                        "[ZeroMode] Done. Offsets written to CANcoder flash."));

        var seqSimDisabled =
                Commands.print("[ZeroMode] Skipped in simulation: no real CAN / no FLASH writes.");

        // Bind once, schedule two commands on the same trigger:
        // run zero sequence (real vs sim)
        var disabledZero = RobotModeTriggers.disabled().and(zeroHold);
        disabledZero.onTrue(Commands.either(seqRealDisabled, seqSimDisabled, RobotBase::isReal));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Path");
    }
}
