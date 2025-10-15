// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.util.CalibrateAzimuthPersist;
import frc.robot.subsystems.util.Haptics;
import frc.robot.subsystems.util.JerkSnapLimiter4th;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double deadBandJoyStick = 0.06;

    // 4th-order limiters for joystick → smooth vx, vy, omega
    private final JerkSnapLimiter4th vxLim =
    new JerkSnapLimiter4th(16.0, 64.0, 800.0, 8000.0, 0.12, 0.0);
    private final JerkSnapLimiter4th vyLim =
    new JerkSnapLimiter4th(16.0, 64.0, 800.0, 8000.0, 0.12, 0.0);
    private final JerkSnapLimiter4th omLim =
    new JerkSnapLimiter4th(12.0, 48.0, 600.0, 6000.0, 0.16, 0.0); // yaw a touch smoother

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Default drive: joystick → 4th-order filtered → robot units
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
              // 0) Pre-deadband (already in your code), then hard quiet zone to kill tiny noise
              final double db = deadBandJoyStick;
              final double stickQuiet = 0.03;   // ≈3% stick; adjust 0.02–0.05
              final double outQuiet   = 0.015;  // post-filter floor
          
              double rawVX = quiet(-MathUtil.applyDeadband(joystick.getLeftY(),  db), stickQuiet);
              double rawVY = quiet(-MathUtil.applyDeadband(joystick.getLeftX(),  db), stickQuiet);
              double rawOM = quiet(-MathUtil.applyDeadband(joystick.getRightX(), db), stickQuiet);
          
              // 1) 4th-order smoothing
              double vxUnit = vxLim.calculate(rawVX);
              double vyUnit = vyLim.calculate(rawVY);
              double omUnit = omLim.calculate(rawOM);
          
              // 2) Zero-hold: if target is zero and filter output is tiny, snap state to zero
              if (rawVX == 0.0 && Math.abs(vxUnit) < outQuiet) { vxLim.reset(0.0); vxUnit = 0.0; }
              if (rawVY == 0.0 && Math.abs(vyUnit) < outQuiet) { vyLim.reset(0.0); vyUnit = 0.0; }
              if (rawOM == 0.0 && Math.abs(omUnit) < outQuiet) { omLim.reset(0.0); omUnit = 0.0; }
          
              // 3) Optional final tiny floor (prevents dithering in sim)
              vxUnit = MathUtil.applyDeadband(vxUnit, outQuiet);
              vyUnit = MathUtil.applyDeadband(vyUnit, outQuiet);
              omUnit = MathUtil.applyDeadband(omUnit, outQuiet);
          
              return drive
                  .withVelocityX(vxUnit * MaxSpeed)
                  .withVelocityY(vyUnit * MaxSpeed)
                  .withRotationalRate(omUnit * MaxAngularRate);
            })
          );

        // Hold Right Bumper to bypass the 4th-order filter (raw drive)
        joystick.rightBumper().whileTrue(
            drivetrain.applyRequest(() ->
            drive.withVelocityX(-MathUtil.applyDeadband(joystick.getLeftY(),  deadBandJoyStick) * MaxSpeed)
                .withVelocityY(-MathUtil.applyDeadband(joystick.getLeftX(),  deadBandJoyStick) * MaxSpeed)
                .withRotationalRate(-MathUtil.applyDeadband(joystick.getRightX(), deadBandJoyStick) * MaxAngularRate))
        );
      
        // Idle while Disabled (keeps neutral mode applied)
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
      
        // Reset filters when entering Disabled (prevents carry-over feel)
        RobotModeTriggers.disabled().onTrue(
            Commands.runOnce(() -> {
              vxLim.reset(-MathUtil.applyDeadband(joystick.getLeftY(),  deadBandJoyStick));
              vyLim.reset(-MathUtil.applyDeadband(joystick.getLeftX(),  deadBandJoyStick));
              omLim.reset(-MathUtil.applyDeadband(joystick.getRightX(), deadBandJoyStick));
            }).ignoringDisable(true)
          );          
      
        // SysId routines
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
      
        // Reset field-centric heading
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
      
        // Telemetry
        drivetrain.registerTelemetry(logger::telemeterize);
      
        // Zero-mode: hold X+Y+A+B for 3s (Disabled-only); real vs sim branches + rumble
        var zeroCombo = joystick.x().and(joystick.y()).and(joystick.a()).and(joystick.b()).debounce(3.0);
      
        var seqRealDisabled =
            Commands.print("[ZeroMode] Starting (DISABLED)…")
                .andThen(new CalibrateAzimuthPersist().ignoringDisable(true))
                .andThen(Commands.print("[ZeroMode] Done. Offsets written to CANcoder flash."));
      
        var seqSimDisabled =
            Commands.print("[ZeroMode] Skipped in simulation: no real CAN / no FLASH writes.");
      
        var disabledZero = RobotModeTriggers.disabled().and(zeroCombo);
        disabledZero.onTrue(Commands.either(seqRealDisabled, seqSimDisabled, RobotBase::isReal));
        disabledZero.onTrue(Haptics.buzzOK(joystick));
      }
    
    // Helper used by drive mapping
    private static double quiet(double x, double eps) {
        return Math.abs(x) < eps ? 0.0 : x;
    }
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
