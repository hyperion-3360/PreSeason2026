// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.util.CalibrateAzimuthPersist;
import frc.robot.subsystems.util.Diagnostics;
import frc.robot.subsystems.util.Haptics;

public class RobotContainer {
    private double MaxSpeed =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =
            RotationsPerSecond.of(0.75)
                    .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double deadBandJoyStick = 0.06;

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

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();

        Diagnostics.bootDiagOne(
                "FL", RobotConfig.flEnc(), RobotConfig.flSteer(), RobotConfig.flDrive());
        Diagnostics.bootDiagOne(
                "FR", RobotConfig.frEnc(), RobotConfig.frSteer(), RobotConfig.frDrive());
        Diagnostics.bootDiagOne(
                "BL", RobotConfig.blEnc(), RobotConfig.blSteer(), RobotConfig.blDrive());
        Diagnostics.bootDiagOne(
                "BR", RobotConfig.brEnc(), RobotConfig.brSteer(), RobotConfig.brDrive());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive.withVelocityX(
                                                -MathUtil.applyDeadband(
                                                                joystick.getLeftY(),
                                                                deadBandJoyStick)
                                                        * MaxSpeed)
                                        .withVelocityY(
                                                -MathUtil.applyDeadband(
                                                                joystick.getLeftX(),
                                                                deadBandJoyStick)
                                                        * MaxSpeed)
                                        .withRotationalRate(
                                                -MathUtil.applyDeadband(
                                                                joystick.getRightX(),
                                                                deadBandJoyStick)
                                                        * MaxAngularRate)));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled()
                .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // Might be used later, keeping it here for example.
        /*joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));*/

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start()
                .and(joystick.y())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start()
                .and(joystick.x())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

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
