package frc.robot.subsystems.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Persists CANcoder MagnetOffset to device FLASH so that the current absolute angle becomes zero.
 * Intended to be run manually (hold X+Y+A+B for 3s) with wheels set to your mechanical zero.
 *
 * <p>Validation: - Reads OLD MagnetOffset from FLASH (old zero), computes NEW = -absNow (mod 1),
 * persists it, reads back to verify within 1 LSB, and then checks absolute angles post-write are ~0
 * within 2 LSBs. - Skips entirely in desktop simulation (no real CAN / no FLASH).
 */
public class CalibrateAzimuthPersist extends Command {

    // Real CANcoder IDs from your project (default CAN bus).
    // If you're on a Canivore, change to: new CANcoder(id, "canivore")
    private final CANcoder fl = new CANcoder(16); // Front Left
    private final CANcoder fr = new CANcoder(14); // Front Right
    private final CANcoder rl = new CANcoder(13); // Rear  Left
    private final CANcoder rr = new CANcoder(15); // Rear  Right

    private boolean done = false;

    // One LSB of CANcoder absolute (turns): 1/4096 ≈ 0.0002441406 turns
    private static final double kOneLsbTurns = 1.0 / 4096.0;
    // Accept ≤ 1 LSB difference for write/read-back verification
    private static final double kWriteVerifyEps = kOneLsbTurns + 1e-6;
    // Accept ≤ 2 LSBs for post-write absolute ~0 check
    private static final double kAbsZeroCheckEps = (2.0 * kOneLsbTurns) + 1e-6;
    // Small settle time to allow FLASH commit before read-back
    private static final double kFlashSettleSec = 0.05;

    @Override
    public void initialize() {
        if (!RobotBase.isReal()) {
            System.out.println("[ZeroMode] Skipping in simulation: no real CAN / no FLASH writes.");
            done = true;
            return;
        }

        System.out.println("[SwerveCal] === Begin azimuth calibration to CANcoder FLASH ===");
        done = false;

        // --- 1) Read OLD offsets (from FLASH) and current absolute angles (normalized) ---
        CANcoderConfiguration flOld = refresh(fl);
        CANcoderConfiguration frOld = refresh(fr);
        CANcoderConfiguration rlOld = refresh(rl);
        CANcoderConfiguration rrOld = refresh(rr);

        double flOldOffset = normTurns(flOld.MagnetSensor.MagnetOffset);
        double frOldOffset = normTurns(frOld.MagnetSensor.MagnetOffset);
        double rlOldOffset = normTurns(rlOld.MagnetSensor.MagnetOffset);
        double rrOldOffset = normTurns(rrOld.MagnetSensor.MagnetOffset);

        double flAbsNow = normTurns(fl.getAbsolutePosition().getValueAsDouble());
        double frAbsNow = normTurns(fr.getAbsolutePosition().getValueAsDouble());
        double rlAbsNow = normTurns(rl.getAbsolutePosition().getValueAsDouble());
        double rrAbsNow = normTurns(rr.getAbsolutePosition().getValueAsDouble());

        // --- 2) Compute NEW offsets so current absolute becomes zero: new = -absNow (mod 1) ---
        double flNewOffset = normTurns(-flAbsNow);
        double frNewOffset = normTurns(-frAbsNow);
        double rlNewOffset = normTurns(-rlAbsNow);
        double rrNewOffset = normTurns(-rrAbsNow);

        // --- 3) Persist NEW offsets to FLASH (handle OK_ButExpectCommLoss as success) ---
        StatusCode sFL = applyOffsetPersist(fl, flNewOffset);
        StatusCode sFR = applyOffsetPersist(fr, frNewOffset);
        StatusCode sRL = applyOffsetPersist(rl, rlNewOffset);
        StatusCode sRR = applyOffsetPersist(rr, rrNewOffset);

        // Allow FLASH to commit before verifying
        Timer.delay(kFlashSettleSec);

        // --- 4) Read back offsets from FLASH and verify within 1 LSB ---
        double flReadBack = normTurns(refresh(fl).MagnetSensor.MagnetOffset);
        double frReadBack = normTurns(refresh(fr).MagnetSensor.MagnetOffset);
        double rlReadBack = normTurns(refresh(rl).MagnetSensor.MagnetOffset);
        double rrReadBack = normTurns(refresh(rr).MagnetSensor.MagnetOffset);

        logModule("FL", flAbsNow, flOldOffset, flNewOffset, flReadBack, sFL);
        logModule("FR", frAbsNow, frOldOffset, frNewOffset, frReadBack, sFR);
        logModule("RL", rlAbsNow, rlOldOffset, rlNewOffset, rlReadBack, sRL);
        logModule("RR", rrAbsNow, rrOldOffset, rrNewOffset, rrReadBack, sRR);

        verifyWritten("FL", flNewOffset, flReadBack);
        verifyWritten("FR", frNewOffset, frReadBack);
        verifyWritten("RL", rlNewOffset, rlReadBack);
        verifyWritten("RR", rrNewOffset, rrReadBack);

        // --- 5) Post-write absolute check: abs should be ~0 (mod 1) within 2 LSBs ---
        double flAbsAfter = normTurns(fl.getAbsolutePosition().getValueAsDouble());
        double frAbsAfter = normTurns(fr.getAbsolutePosition().getValueAsDouble());
        double rlAbsAfter = normTurns(rl.getAbsolutePosition().getValueAsDouble());
        double rrAbsAfter = normTurns(rr.getAbsolutePosition().getValueAsDouble());

        verifyAbsZero("FL", flAbsAfter);
        verifyAbsZero("FR", frAbsAfter);
        verifyAbsZero("RL", rlAbsAfter);
        verifyAbsZero("RR", rrAbsAfter);

        System.out.println("[SwerveCal] === Done. Offsets written to FLASH. Reboot optional. ===");
        done = true;
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    // ---------- Helpers ----------

    /** Pulls current config from device FLASH into a fresh config object. */
    private static CANcoderConfiguration refresh(CANcoder enc) {
        CANcoderConfiguration cfg = new CANcoderConfiguration();
        enc.getConfigurator().refresh(cfg);
        return cfg;
    }

    /** Normalize a value in turns to the range [0, 1). */
    private static double normTurns(double x) {
        double y = x % 1.0;
        if (y < 0) y += 1.0;
        return y;
    }

    /** Minimal signed difference (-0.5, +0.5] in turns, useful for modulo comparisons. */
    private static double wrapSymmetricTurns(double x) {
        double y = x % 1.0;
        if (y <= -0.5) y += 1.0;
        if (y > 0.5) y -= 1.0;
        return y;
    }

    /** Persist MagnetOffset to FLASH; portable across Phoenix6 versions. */
    private static StatusCode applyOffsetPersist(CANcoder enc, double offsetTurns) {
        CANcoderConfiguration cfg = new CANcoderConfiguration();
        enc.getConfigurator().refresh(cfg); // start from current config
        cfg.MagnetSensor.MagnetOffset = offsetTurns; // set new offset
        StatusCode sc = enc.getConfigurator().apply(cfg); // persist to FLASH

        // Use generic API so this compiles everywhere, and still logs exactly what happened.
        String name = sc.getName();
        if (sc.isError()) {
            System.out.printf("[SwerveCal][ERROR] FLASH write status: %s%n", name);
        } else if (sc.isWarning()) {
            // FLASH commits often cause a brief comm warning; that's expected.
            System.out.printf(
                    "[SwerveCal][WARN] FLASH write status: %s (expected during FLASH commit)%n",
                    name);
        } else {
            System.out.printf("[SwerveCal] FLASH write status: %s%n", name);
        }

        return sc;
    }

    /** Logs one module's details: abs-now, old/new offsets, read-back, delta, and status. */
    private static void logModule(
            String name,
            double absNow,
            double oldOff,
            double newOff,
            double readBack,
            StatusCode st) {
        double deltaOldToNew = wrapSymmetricTurns(newOff - oldOff);
        System.out.printf(
                "[SwerveCal] %s absNow=%.6f oldOffset=%.6f newOffset=%.6f readBack=%.6f Δ(old→new)=%.6f (%s)%n",
                name, absNow, oldOff, newOff, readBack, deltaOldToNew, st);
    }

    /** Verifies requested vs. read-back offset within 1 LSB. */
    private static void verifyWritten(String name, double requested, double readBack) {
        double diff = Math.abs(wrapSymmetricTurns(readBack - requested));
        if (diff > kWriteVerifyEps) {
            System.out.printf(
                    "[SwerveCal][WARN] %s read-back mismatch: requested=%.6f readBack=%.6f diff=%.6f (turns)%n",
                    name, requested, readBack, diff);
        } else {
            System.out.printf(
                    "[SwerveCal] %s persisted OK (|diff|=%.6f ≤ %.6f ~ 1 LSB).%n",
                    name, diff, kWriteVerifyEps);
        }
    }

    /** Verifies that absolute angle after write is ~0 (mod 1) within 2 LSBs. */
    private static void verifyAbsZero(String name, double absAfter) {
        double distToZero = Math.min(absAfter, 1.0 - absAfter); // distance to nearest integer turn
        if (distToZero > kAbsZeroCheckEps) {
            System.out.printf(
                    "[SwerveCal][WARN] %s abs-after not ~0: abs=%.6f distToZero=%.6f (> %.6f)%n",
                    name, absAfter, distToZero, kAbsZeroCheckEps);
        } else {
            System.out.printf(
                    "[SwerveCal] %s abs-after ~0 OK (abs=%.6f, distToZero=%.6f ≤ %.6f).%n",
                    name, absAfter, distToZero, kAbsZeroCheckEps);
        }
    }
}
