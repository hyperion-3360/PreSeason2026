// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.util.Scorekeeper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class Scorekeeper extends SubsystemBase {
    private CommandSwerveDrivetrain m_drivetrain;
    private double robotX, robotY;

    public Scorekeeper() {}

    @Override
    public void periodic() {
        robotX = m_drivetrain.getState().Pose.getX();
        robotY = m_drivetrain.getState().Pose.getY();
        System.out.println(robotX + ", " + robotY);
    }
}
