package frc.robot.subsystems.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * 4th-order input limiter with C³ continuity.
 * State: position y, velocity v, acceleration a, jerk j.
 * Limits: |v| ≤ vmax, |a| ≤ amax, |j| ≤ jmax, and |snap| ≤ smax (snap = dj/dt).
 *
 * Use it on joystick inputs to remove step-yank, limit jerk, and make jerk itself smooth.
 * Typical usage (20ms loop): call calculate(target) each loop, feed result to your drive.
 */
public final class JerkSnapLimiter4th {
  // Tunables (units are "input units" per second^N)
  private final double vmax;   // max rate dy/dt
  private final double amax;   // max accel dv/dt
  private final double jmax;   // max jerk  da/dt
  private final double smax;   // max snap  dj/dt
  private final double kVel;   // proportional map from position error -> desired velocity (1/sec)

  // State
  private double y;  // output (filtered position, same units as input)
  private double v;  // dy/dt
  private double a;  // dv/dt
  private double j;  // da/dt

  private double lastTs = Double.NaN;

  /**
   * @param vmax max velocity (units/s)
   * @param amax max acceleration (units/s^2)
   * @param jmax max jerk (units/s^3)
   * @param smax max snap (units/s^4)
   * @param velocityResponseTimeSec time constant to turn position error into a velocity demand (≈ how "eager" it is)
   * @param initialValue initial output y (usually your current joystick reading or 0)
   */
  public JerkSnapLimiter4th(double vmax, double amax, double jmax, double smax,
                            double velocityResponseTimeSec, double initialValue) {
    this.vmax = Math.abs(vmax);
    this.amax = Math.abs(amax);
    this.jmax = Math.abs(jmax);
    this.smax = Math.abs(smax);
    this.kVel = 1.0 / Math.max(1e-6, velocityResponseTimeSec);
    reset(initialValue);
  }

  /** Convenience: default initial value 0. */
  public JerkSnapLimiter4th(double vmax, double amax, double jmax, double smax, double velocityResponseTimeSec) {
    this(vmax, amax, jmax, smax, velocityResponseTimeSec, 0.0);
  }

  /** Reset state to a specific output value (zero v/a/j). */
  public void reset(double outputValue) {
    this.y = outputValue;
    this.v = 0.0;
    this.a = 0.0;
    this.j = 0.0;
    this.lastTs = Timer.getFPGATimestamp();
  }

  /** Current output value. */
  public double get() { return y; }

  /**
   * One-step update using FPGA time for dt (20 ms typical).
   * @param target desired input (same units as output), e.g., joystick value in [-1..1]
   * @return filtered output
   */
  public double calculate(double target) {
    double now = Timer.getFPGATimestamp();
    double dt = now - (Double.isNaN(lastTs) ? now : lastTs);
    lastTs = now;
    // clamp weird dt spikes
    if (dt <= 0.0 || dt > 0.1) dt = 0.02;
    return calculate(target, dt);
  }

  /**
   * One-step update with explicit dt.
   * @param target desired input (same units as output)
   * @param dt seconds since last call
   * @return filtered output
   */
  public double calculate(double target, double dt) {
    // 1) Position error
    double e = target - y;

    // 2) Desired velocity from position error, limited
    double vDes = MathUtil.clamp(kVel * e, -vmax, vmax);

    // 3) Desired acceleration from velocity change, limited
    double aDes = MathUtil.clamp((vDes - v) / dt, -amax, amax);

    // 4) Desired jerk from acceleration change, limited
    double jDes = MathUtil.clamp((aDes - a) / dt, -jmax, jmax);

    // 5) SNAP-limit the change in jerk (4th derivative)
    double snapCmd = MathUtil.clamp((jDes - j) / dt, -smax, smax);

    // 6) Integrate snap→jerk→accel→vel→pos
    j += snapCmd * dt;
    j = MathUtil.clamp(j, -jmax, jmax);   // <— add this line

    a += j * dt;
    a = MathUtil.clamp(a, -amax, amax);

    v += a * dt;
    v = MathUtil.clamp(v, -vmax, vmax);

    // Optional: keep y in joystick range if you’re using [-1..1] everywhere
    y = MathUtil.clamp(y, -1.0, 1.0);

    return y;
  }
}
