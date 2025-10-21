package frc.robot.subsystems.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/** Jerk-limited S-curve on velocity. */
public final class SCurveLimiter {
    private double v, a;
    private final double vmax, amax, jmax;
    private double lastTs = Double.NaN;

    // guardrails for dt
    private static final double DT_MIN = 0.002; // 2 ms
    private static final double DT_MAX = 0.100; // 100 ms

    public SCurveLimiter(double vmax, double amax, double jmax) {
        this.vmax = vmax; // joystick units / s (use 1.0 for full scale)
        this.amax = amax; // joystick units / s^2
        this.jmax = jmax; // joystick units / s^3
    }

    /** Reset to a known output velocity (usually 0). */
    public void reset(double v0) {
        v = v0;
        a = 0;
        lastTs = Double.NaN;
    }

    /** One-step update using FPGA time. */
    public double calculate(double target) {
        double now = Timer.getFPGATimestamp();
        double dt = (Double.isNaN(lastTs) ? 0.02 : now - lastTs);
        lastTs = now;
        if (dt < DT_MIN || dt > DT_MAX) dt = 0.02; // clamp weird timestamps
        return calculate(target, dt);
    }

    /** One-step update with explicit dt. */
    public double calculate(double target, double dt) {
        // 1) Clamp target within our velocity limit
        double vCmd = MathUtil.clamp(target, -vmax, vmax);

        // 2) Accel needed to chase vCmd this step, accel-limited
        double aDes = MathUtil.clamp((vCmd - v) / dt, -amax, amax);

        // 3) Jerk-limit accel change toward aDes
        double da = MathUtil.clamp(aDes - a, -jmax * dt, jmax * dt);
        a += da;

        // 4) Integrate accel -> velocity
        v += a * dt;

        // Anti-windup at saturation: if pegged and still pushing, stop accelerating
        if ((v >= 0.999 * vmax && a > 0) || (v <= -0.999 * vmax && a < 0)) {
            v = MathUtil.clamp(v, -vmax, vmax);
            a = 0;
        }

        // 5) Snap-to-zero to kill micro-oscillations (scale with limits)
        double epsIn = 1e-3 * vmax;
        double epsOut = 2e-3 * vmax;
        if (Math.abs(vCmd) < epsIn && Math.abs(v) < epsOut) {
            v = 0;
            a = 0;
        }

        return v;
    }

    // expose for logging/tuning
    public double v() {
        return v;
    }

    public double a() {
        return a;
    }
}
