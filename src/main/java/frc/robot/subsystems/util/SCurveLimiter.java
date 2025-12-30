package frc.robot.subsystems.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/** Jerk-limited S-curve on velocity. */
public final class SCurveLimiter {
    private double v, a;
    private final double vmax, amax, jmax;
    private double lastTs = Double.NaN;

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
        double dt =
                (Double.isNaN(lastTs) ? Constants.DriveConstants.SCURVE_DT_DEFAULT : now - lastTs);
        lastTs = now;
        if (dt < Constants.DriveConstants.SCURVE_DT_MIN
                || dt > Constants.DriveConstants.SCURVE_DT_MAX)
            dt = Constants.DriveConstants.SCURVE_DT_DEFAULT; // clamp weird timestamps
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
        // Uses configurable threshold from Constants to prevent overshoot
        double saturatedPosThreshold =
                Constants.DriveConstants.SCURVE_ANTIWINDUP_THRESHOLD * vmax;
        double saturatedNegThreshold =
                -Constants.DriveConstants.SCURVE_ANTIWINDUP_THRESHOLD * vmax;

        if ((v >= saturatedPosThreshold && a > 0) || (v <= saturatedNegThreshold && a < 0)) {
            v = MathUtil.clamp(v, -vmax, vmax);
            a = 0;
        }

        // 5) Snap-to-zero to kill micro-oscillations (scale with limits)
        double epsIn = Constants.DriveConstants.SCURVE_SNAP_INPUT_THRESHOLD * vmax;
        double epsOut = Constants.DriveConstants.SCURVE_SNAP_OUTPUT_THRESHOLD * vmax;
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
