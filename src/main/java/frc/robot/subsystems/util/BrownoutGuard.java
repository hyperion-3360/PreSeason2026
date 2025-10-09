package frc.robot.subsystems.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * BrownoutGuard
 * --------------
 * Purpose:
 *   Softly protect the robot when the battery voltage sags ("brownout") by
 *   scaling down drive commands and optionally triggering callbacks (rumble,
 *   neutral-mode changes, logging).
 *
 * How it works:
 *   - Monitors the supplied battery voltage each loop via update(vbat).
 *   - Enters "brownout mode" when voltage < enterVolts for at least enterDebounceSec.
 *   - Exits "brownout mode" when voltage > exitVolts for at least exitDebounceSec.
 *   - Computes a *continuous* scale factor in [minTransScale, 1.0] that you
 *     apply to translation commands (vx, vy). Rotation (omega) gets an extra
 *     bias factor (omegaBias) so you can reduce it a bit more aggressively.
 *
 * Why:
 *   - Keeps the robot controllable when the battery sags under heavy load.
 *   - Prevents abrupt controller resets, CAN dropouts, and brownouts.
 */
public class BrownoutGuard {

  /** Tunable configuration bundle. */
  public static final class Config {
    /** Enter brownout when VBAT falls below this voltage. */
    public double enterVolts = 10.0;
    /** Exit brownout only after VBAT rises above this voltage. */
    public double exitVolts  = 10.5;

    /** Minimum translation scale at/under (enterVolts - 0.5V). Range [0..1]. */
    public double minTransScale = 0.50;
    /**
     * Additional reduction applied to rotation (omega). 0.80 means
     * omega is scaled by (0.80 * translationScale).
     */
    public double omegaBias = 0.80;

    /** Debounce time required to enter brownout after crossing enterVolts. */
    public double enterDebounceSec = 0.20;
    /** Debounce time required to exit brownout after crossing exitVolts. */
    public double exitDebounceSec  = 0.40;

    /**
     * Width of the linear scaling ramp below exitVolts.
     * Scale goes from 1.0 at exitVolts down to minTransScale at (enterVolts - rampSpanVolts).
     * Default = 1.0V → example: 10.5V → 9.5V.
     */
    public double rampSpanVolts = 1.0;
  }

  private final Config cfg;

  /** True when in brownout mode (after debounce). */
  private boolean inBrownout = false;

  /** Last time (FMS clock) we toggled brownout state, used for debounce. */
  private double lastStateChangeSec = 0.0;

  /** Current translation scale factor in [minTransScale..1.0]. */
  private double translationScale = 1.0;

  /** Optional user callbacks (e.g., rumble, log, neutral-mode switch). */
  private Runnable onEnter = () -> {};
  private Runnable onExit  = () -> {};

  public BrownoutGuard(Config cfg) {
    this.cfg = cfg;
  }

  // ---------------------------- Public API ----------------------------

  /** Register a callback invoked once when entering brownout mode. */
  public BrownoutGuard onEnterBrownout(Runnable r) { this.onEnter = r != null ? r : () -> {}; return this; }

  /** Register a callback invoked once when exiting brownout mode. */
  public BrownoutGuard onExitBrownout(Runnable r)  { this.onExit  = r != null ? r : () -> {}; return this; }

  /** Returns true if currently in brownout mode (after debounce). */
  public boolean isBrownout() { return inBrownout; }

  /** Scale applied to translation commands (vx, vy). */
  public double getTranslationScale() { return translationScale; }

  /** Scale applied to omega (rotation). Equals translationScale * omegaBias. */
  public double getOmegaScale() { return translationScale * cfg.omegaBias; }

  /**
   * Call once per loop with the *current* battery voltage.
   * Computes the new brownout state (with hysteresis + debounce) and
   * updates the scaling factors. Returns the translation scale for convenience.
   */
  public double update(double vbat) {
    final double now = Timer.getFPGATimestamp();

    // --- Debounced state transitions (hysteresis) ---
    if (!inBrownout) {
      if (vbat < cfg.enterVolts && (now - lastStateChangeSec) >= cfg.enterDebounceSec) {
        inBrownout = true;
        lastStateChangeSec = now;
        onEnter.run();
      }
    } else {
      if (vbat > cfg.exitVolts && (now - lastStateChangeSec) >= cfg.exitDebounceSec) {
        inBrownout = false;
        lastStateChangeSec = now;
        onExit.run();
      }
    }

    // --- Continuous scaling curve (translation) ---
    final double vMax = cfg.exitVolts;                 // scale = 1.0 here and above
    final double vMin = cfg.enterVolts - cfg.rampSpanVolts;  // scale = minTransScale here and below

    if (vbat >= vMax) {
      translationScale = 1.0;
    } else if (vbat <= vMin) {
      translationScale = MathUtil.clamp(cfg.minTransScale, 0.0, 1.0);
    } else {
      // Linear interpolation between [vMin..vMax] → [minTransScale..1.0]
      final double t = (vbat - vMin) / (vMax - vMin);
      translationScale = MathUtil.clamp(cfg.minTransScale + t * (1.0 - cfg.minTransScale), 0.0, 1.0);
    }

    return translationScale;
  }
}
