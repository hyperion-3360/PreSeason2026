package frc.robot.subsystems.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Haptics helper for controller vibration (rumble). Notes: - Intensities are [0.0..1.0] per side. -
 * Works reliably on real robot (Driver Station). Desktop sim may not rumble.
 */
public final class Haptics {
    private Haptics() {}

    /**
     * Rumble both motors for a fixed duration.
     *
     * @param pad Xbox controller (command wrapper)
     * @param left left motor intensity [0..1]
     * @param right right motor intensity [0..1]
     * @param seconds duration in seconds (>0)
     * @return a command that starts rumble on schedule and stops on end/timeout
     */
    public static Command rumble(
            CommandXboxController pad, double left, double right, double seconds) {
        return Commands.runEnd(
                        () -> {
                            pad.getHID().setRumble(RumbleType.kLeftRumble, left);
                            pad.getHID().setRumble(RumbleType.kRightRumble, right);
                        },
                        () -> {
                            pad.getHID().setRumble(RumbleType.kLeftRumble, 0);
                            pad.getHID().setRumble(RumbleType.kRightRumble, 0);
                        })
                .withTimeout(seconds);
    }

    /** Short success buzz (balanced). */
    public static Command buzzOK(CommandXboxController pad) {
        return rumble(pad, 0.7, 0.7, 0.30);
    }

    /** Very short single buzz. */
    public static Command buzzShort(CommandXboxController pad) {
        return rumble(pad, 0.5, 0.5, 0.15);
    }
}
