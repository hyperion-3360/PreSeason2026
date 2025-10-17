package frc.robot.subsystems.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * Jerk-limited S-curve on VELOCITY. Input/Output unit is a normalized joystick value in [-1..1].
 * State: current velocity v and acceleration a. Limits: vmax (|v|), amax (|a|), jmax (|da/dt|).
 */
public final class SCurveLimiter {
    private double v, a;
    private final double vmax, amax, jmax;
    private double lastTs = Double.NaN;

    public SCurveLimiter(double vmax, double amax, double jmax) {
        this.vmax = vmax; // units/s in joystick units (1.0 == full scale)
        this.amax = amax; // units/s^2
        this.jmax = jmax; // units/s^3
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
        if (dt <= 0.0 || dt > 0.1) dt = 0.02;
        return calculate(target, dt);
    }

    /** One-step update with explicit dt. */
    public double calculate(double target, double dt) {
        // Clamp the target into our limits
        double vCmd = MathUtil.clamp(target, -vmax, vmax);

        // Desired accel to reach the target velocity this step (bounded)
        double aDes = MathUtil.clamp((vCmd - v) / dt, -amax, amax);

        // Jerk-limit the accel change toward aDes
        double da = MathUtil.clamp(aDes - a, -jmax * dt, jmax * dt);
        a += da;

        // Integrate accel -> velocity
        v += a * dt;

        // Anti-windup at saturation: if we are stuck and we continue to “push”, zero acceleration
        if ((v >= 0.999 * vmax && a > 0) || (v <= -0.999 * vmax && a < 0)) {
            v = MathUtil.clamp(v, -vmax, vmax);
            a = 0;
        }

        // Snap-to-zero when both target and output are tiny to kill micro-oscillations
        if (Math.abs(vCmd) < 1e-3 && Math.abs(v) < 2e-3) {
            v = 0;
            a = 0;
        }
        return v;
    }

    // Optional: expose for logging/tuning
    public double v() {
        return v;
    }

    public double a() {
        return a;
    }
}
