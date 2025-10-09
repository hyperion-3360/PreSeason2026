package frc.robot.subsystems.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class Haptics {
  private Haptics() {}

  public static Command rumble(CommandXboxController pad, double left, double right, double seconds) {
    return Commands.startEnd(
        () -> { pad.getHID().setRumble(RumbleType.kLeftRumble, left);
                pad.getHID().setRumble(RumbleType.kRightRumble, right); },
        () -> { pad.getHID().setRumble(RumbleType.kLeftRumble, 0);
                pad.getHID().setRumble(RumbleType.kRightRumble, 0); }
    ).withTimeout(seconds);
  }

  public static Command buzzOK(CommandXboxController pad) { return rumble(pad, 0.7, 0.7, 0.30); }
}
