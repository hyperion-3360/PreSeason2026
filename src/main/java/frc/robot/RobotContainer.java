package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.commands.AlignToTagCommand;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.util.BrownoutProtection;
import frc.robot.subsystems.util.CalibrateAzimuthPersist;
import frc.robot.subsystems.util.Diagnostics;
import frc.robot.subsystems.util.ExponentialScale;
import frc.robot.subsystems.util.Haptics;
import frc.robot.subsystems.util.SCurveLimiter;
import frc.robot.subsystems.util.SnapTo;
import frc.robot.subsystems.vision.VisionSubsystem;
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

    // Auto-aim PID controller
    private final PIDController autoAimPID =
            new PIDController(
                    Constants.AutoAlignConstants.AUTO_AIM_kP,
                    Constants.AutoAlignConstants.AUTO_AIM_kI,
                    Constants.AutoAlignConstants.AUTO_AIM_kD);

    {
        // Configure auto-aim PID
        autoAimPID.enableContinuousInput(-Math.PI, Math.PI);
        autoAimPID.setTolerance(Constants.AutoAlignConstants.AUTO_AIM_TOLERANCE);
    }

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(MaxSpeed * 0.1)
                    .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(
                            DriveRequestType
                                    .OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final VisionSubsystem vision = new VisionSubsystem(drivetrain);

    private final SnapTo m_snapTo = new SnapTo(drivetrain);

    private boolean m_snappingTo = false;
    private double m_snapToSetpoint;

    private final PIDController m_snapToPid =
            new PIDController(
                    Constants.SnapToConstants.kP,
                    Constants.SnapToConstants.kI,
                    Constants.SnapToConstants.kD);

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

        m_snapToPid.setTolerance(1);
    }

    /** Called periodically to update battery voltage monitoring and logging */
    public void periodic() {
        brownoutProtection.update();

        // Log driver inputs (raw, after deadband)
        Logger.recordOutput("Driver/LeftY", joystick.getLeftY());
        Logger.recordOutput("Driver/LeftX", joystick.getLeftX());
        Logger.recordOutput("Driver/RightX", joystick.getRightX());
        Logger.recordOutput("Driver/LeftTrigger", joystick.getLeftTriggerAxis());
        Logger.recordOutput("Driver/RightTrigger", joystick.getRightTriggerAxis());

        // Log control modes
        Logger.recordOutput("RobotState/SCurveEnabled", sCurveEnabled);
        Logger.recordOutput("RobotState/AutoAimEnabled", autoAimEnabled);

        // Log battery and brownout protection
        Logger.recordOutput("Battery/Voltage", brownoutProtection.getVoltage());
        Logger.recordOutput("Battery/Status", brownoutProtection.getStatus().toString());
        Logger.recordOutput("Battery/SpeedScale", brownoutProtection.getSpeedScaleFactor());
        Logger.recordOutput("Battery/IsCritical", brownoutProtection.isCritical());

        // Log max speeds (useful for seeing brownout effects)
        Logger.recordOutput("RobotState/MaxSpeed", MaxSpeed);
        Logger.recordOutput("RobotState/MaxAngularRate", MaxAngularRate);
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

                            // Apply brownout protection to translation
                            double speedScale = brownoutProtection.getSpeedScaleFactor();
                            xCmd *= speedScale;
                            yCmd *= speedScale;

                            // Auto-aim mode: robot controls rotation, driver controls translation
                            if (autoAimEnabled && vision.hasTarget()) {
                                Rotation2d currentHeading =
                                        drivetrain.getState().Pose.getRotation();
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
                            } else {
                                rCmd *= speedScale;
                            }
                            if (!m_snappingTo) {

                                m_snapToPid.reset();
                            }
                            if (m_snappingTo) {

                                double currentpose = m_snapTo.getCurrentAngle().getDegrees() + 180;
                                int inverted = (int) currentpose / 180;

                                if (inverted < 0) {

                                    double difference = 360 - currentpose;
                                    rCmd =
                                            MathUtil.applyDeadband(
                                                    m_snapToPid.calculate(
                                                            difference, m_snapToSetpoint),
                                                    Constants.SnapToConstants.kSnapToDeadband);

                                } else {
                                    rCmd =
                                            MathUtil.applyDeadband(
                                                    m_snapToPid.calculate(
                                                            currentpose, m_snapToSetpoint),
                                                    Constants.SnapToConstants.kSnapToDeadband);

                                    //     rCmd = rCmd * -1;
                                    // }

                                    System.out.println(
                                            "current pose : "
                                                    + m_snapTo.getCurrentAngle().getDegrees());

                                    System.out.println("rCmd value : " + rCmd);

                                    if (m_snapToPid.atSetpoint()) {

                                        m_snappingTo = false;
                                    }
                                }
                            }

                            return drive.withVelocityX(xCmd)
                                    .withVelocityY(yCmd)
                                    .withRotationalRate(rCmd);
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
                .whileTrue(AlignToTagCommand.withDefaultDistance(drivetrain, vision))
                .onFalse(
                        Commands.runOnce(
                                () -> System.out.println("[Align] Button released"), drivetrain));

        // ========== AUTO-AIM TOGGLE ==========
        // Hold both triggers (L2 + R2) for 3 seconds to toggle auto-aim mode
        var bothTriggersPressed = joystick.leftTrigger().and(joystick.rightTrigger());

        // Debug: print when triggers are pressed and give haptic feedback
        bothTriggersPressed.onTrue(
                Commands.runOnce(
                        () -> {
                            System.out.println(
                                    "[Auto-Aim] Both triggers pressed - holding for 3 seconds...");
                            Haptics.buzzShort(joystick).schedule();
                        }));

        // Toggle after 3 second hold
        bothTriggersPressed
                .debounce(3.0)
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    autoAimEnabled = !autoAimEnabled;
                                    if (autoAimEnabled) {
                                        autoAimPID.reset();
                                        System.out.println(
                                                "[Auto-Aim] ENABLED - Robot will auto-rotate to closest AprilTag");
                                        Haptics.buzzOK(joystick).schedule();
                                    } else {
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

        // ========== SYSID ROUTINES ==========
        // Run SysId routines when holding back/start and X/Y.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start()
                .and(joystick.y())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start()
                .and(joystick.x())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // ========== FIELD-CENTRIC RESET ==========
        // Press L1 (left bumper) twice quickly to reset field-centric heading
        // (Double-tap prevents accidental resets)
        joystick.leftBumper()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    drivetrain.seedFieldCentric();
                                    System.out.println(
                                            "[Field-Centric] RESET - Robot forward is now field forward");
                                    Haptics.buzzOK(joystick).schedule();
                                }));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.b()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    m_snappingTo = true;
                                    m_snapToSetpoint = 90;
                                }));

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
        return Commands.print("No autonomous command configured");
    }
}
