package frc.robot.subsystems.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConfig;

/**
 * Persists CANcoder MagnetOffset to device FLASH so that the current absolute angle becomes zero.
 * Intended to be run manually (hold X+Y+A+B for 3s) with wheels set to your mechanical zero.
 *
 * <p>Validation: - Reads OLD MagnetOffset from FLASH (old zero), computes NEW = -absNow (mod 1),
 * persists it, reads back to verify within 1 LSB, and then checks absolute angles post-write are ~0
 * within 2 LSBs. - Skips entirely in desktop simulation (no real CAN / no FLASH).
 */
public class CalibrateAzimuthPersist extends Command {

    // CANcoder IDs from RobotConfig - automatically switches with ACTIVE_SWERVE profile
    // If you're on a Canivore, change to: new CANcoder(id, "canivore")
    private final CANcoder frontLeft = new CANcoder(RobotConfig.flEnc()); // Front Left
    private final CANcoder frontRight = new CANcoder(RobotConfig.frEnc()); // Front Right
    private final CANcoder backLeft = new CANcoder(RobotConfig.blEnc()); // Back Left
    private final CANcoder backRight = new CANcoder(RobotConfig.brEnc()); // Back Right

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
        CANcoderConfiguration frontLeftOldValues = refresh(frontLeft);
        CANcoderConfiguration frontRightOldValues = refresh(frontRight);
        CANcoderConfiguration backLeftOldValues = refresh(backLeft);
        CANcoderConfiguration backRightOldValues = refresh(backRight);

        double frontLeftOldOffset = normTurns(frontLeftOldValues.MagnetSensor.MagnetOffset);
        double frontRightOldOffset = normTurns(frontRightOldValues.MagnetSensor.MagnetOffset);
        double backLeftOldOffset = normTurns(backLeftOldValues.MagnetSensor.MagnetOffset);
        double backRightOldOffset = normTurns(backRightOldValues.MagnetSensor.MagnetOffset);

        double frontLeftAbsNow = normTurns(frontLeft.getAbsolutePosition().getValueAsDouble());
        double frontRearAbsNow = normTurns(frontRight.getAbsolutePosition().getValueAsDouble());
        double backLeftAbsNow = normTurns(backLeft.getAbsolutePosition().getValueAsDouble());
        double backRightAbsNow = normTurns(backRight.getAbsolutePosition().getValueAsDouble());

        // --- 2) Compute NEW offsets so current absolute becomes zero: new = -absNow (modulo 1) ---
        double frontLeftRawNow = normTurns(frontLeftAbsNow - frontLeftOldOffset);
        double frontRightRawNow = normTurns(frontRearAbsNow - frontRightOldOffset);
        double backLeftRawNow = normTurns(backLeftAbsNow - backLeftOldOffset);
        double backRightRawNow = normTurns(backRightAbsNow - backRightOldOffset);

        double frontLeftNewOffset = normTurns(-frontLeftRawNow);
        double frontRightNewOffset = normTurns(-frontRightRawNow);
        double backLeftNewOffset = normTurns(-backLeftRawNow);
        double backRightNewOffset = normTurns(-backRightRawNow);

        // --- 3) Persist NEW offsets to FLASH (handle OK_ButExpectCommLoss as success) ---
        StatusCode statusFrontLeft = applyOffsetPersist(frontLeft, frontLeftNewOffset);
        StatusCode statusFrontRight = applyOffsetPersist(frontRight, frontRightNewOffset);
        StatusCode statusBackLeft = applyOffsetPersist(backLeft, backLeftNewOffset);
        StatusCode statusBackRight = applyOffsetPersist(backRight, backRightNewOffset);

        // Allow FLASH to commit before verifying
        Timer.delay(kFlashSettleSec);

        // --- 5) Read back offsets from FLASH and verify within 1 LSB ---
        double frontLeftReadBack = normTurns(refresh(frontLeft).MagnetSensor.MagnetOffset);
        double frontRightReadBack = normTurns(refresh(frontRight).MagnetSensor.MagnetOffset);
        double backLeftReadBack = normTurns(refresh(backLeft).MagnetSensor.MagnetOffset);
        double backRightReadBack = normTurns(refresh(backRight).MagnetSensor.MagnetOffset);

        logModule(
                "frontLeft",
                frontLeftAbsNow,
                frontLeftOldOffset,
                frontLeftNewOffset,
                frontLeftReadBack,
                statusFrontLeft);
        logModule(
                "frontRight",
                frontRearAbsNow,
                frontRightOldOffset,
                frontRightNewOffset,
                frontRightReadBack,
                statusFrontRight);
        logModule(
                "backLeft",
                backLeftAbsNow,
                backLeftOldOffset,
                backLeftNewOffset,
                backLeftReadBack,
                statusBackLeft);
        logModule(
                "backRight",
                backRightAbsNow,
                backRightOldOffset,
                backRightNewOffset,
                backRightReadBack,
                statusBackRight);

        verifyWritten("frontLeft", frontLeftNewOffset, frontLeftReadBack);
        verifyWritten("frontRight", frontRightNewOffset, frontRightReadBack);
        verifyWritten("backLeft", backLeftNewOffset, backLeftReadBack);
        verifyWritten("backRight", backRightNewOffset, backRightReadBack);

        // --- 5) Post-write absolute check (sensor output ≈ 0 within 2 LSB; works in Disabled) ---
        verifyCanCoderZero("frontLeft", frontLeft);
        verifyCanCoderZero("frontRight", frontRight);
        verifyCanCoderZero("backLeft", backLeft);
        verifyCanCoderZero("backRight", backRight);

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
                "[SwerveCal] %s absNow=%.6f oldOffset=%.6f newOffset=%.6f readBack=%.6f (old→new)=%.6f (%s)%n",
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

    private static void verifyCanCoderZero(String name, CANcoder enc) {
        // Force a fresh read after FLASH write
        StatusSignal<Angle> abs = enc.getAbsolutePosition();
        abs.refresh(); // pull latest frame now
        double v = normTurns(abs.getValueAsDouble()); // [0,1)
        double distToZero = Math.min(v, 1.0 - v); // distance to nearest integer (0 mod 1)
        if (distToZero > kAbsZeroCheckEps) {
            System.out.printf(
                    "[SwerveCal][WARN] %s CANcoder not ~0: abs=%.6f dist=%.6f (> %.6f)%n",
                    name, v, distToZero, kAbsZeroCheckEps);
        } else {
            System.out.printf(
                    "[SwerveCal] %s CANcoder ~0 OK (abs=%.6f, dist=%.6f ≤ %.6f).%n",
                    name, v, distToZero, kAbsZeroCheckEps);
        }
    }
}
