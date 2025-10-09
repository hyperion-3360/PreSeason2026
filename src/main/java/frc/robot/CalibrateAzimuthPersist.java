package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Calibrates each swerve module azimuth by writing CANcoder MagnetOffset
 * to the device's non-volatile FLASH so that the current absolute angle becomes zero.
 *
 * How to use:
 *  1) Manually point each wheel to your mechanical "zero" (e.g., all wheels straight ahead).
 *  2) Hold X + Y + A + B for 3 seconds to run this command.
 *  3) Offsets are persisted to CANcoder FLASH. A reboot is optional but recommended.
 *
 * This version:
 *  - Reads the PREVIOUS MagnetOffset from FLASH (the "old zero").
 *  - Computes the NEW offset from the current absolute angle.
 *  - Writes the NEW offset to FLASH and reads it back to verify persistence.
 *  - Logs deltas so you can compare NEW vs OLD and validate the logic.
 */
public class CalibrateAzimuthPersist extends Command {

  // Real CANcoder IDs from your TunerConstants
  private final CANcoder flCanCoder = new CANcoder(16); // Front Left
  private final CANcoder frCanCoder = new CANcoder(14); // Front Right
  private final CANcoder rlCanCoder = new CANcoder(13); // Back/Rear Left
  private final CANcoder rrCanCoder = new CANcoder(15); // Back/Rear Right

  private boolean done = false;

  // Tolerance for verifying the value written vs. read-back (in turns)
  private static final double kWriteVerifyEps = 1e-4;

  @Override
  public void initialize() {
    System.out.println("[SwerveCal] === Begin azimuth calibration to CANcoder FLASH ===");
    done = false;

    // 1) Read previous MagnetOffsets from FLASH (old zero values),
    //    AND read current absolute positions (already including any existing offset).
    CANcoderConfiguration flCfgOld = new CANcoderConfiguration();
    CANcoderConfiguration frCfgOld = new CANcoderConfiguration();
    CANcoderConfiguration rlCfgOld = new CANcoderConfiguration();
    CANcoderConfiguration rrCfgOld = new CANcoderConfiguration();

    flCanCoder.getConfigurator().refresh(flCfgOld);
    frCanCoder.getConfigurator().refresh(frCfgOld);
    rlCanCoder.getConfigurator().refresh(rlCfgOld);
    rrCanCoder.getConfigurator().refresh(rrCfgOld);

    double flOldOffset = normTurns(flCfgOld.MagnetSensor.MagnetOffset);
    double frOldOffset = normTurns(frCfgOld.MagnetSensor.MagnetOffset);
    double rlOldOffset = normTurns(rlCfgOld.MagnetSensor.MagnetOffset);
    double rrOldOffset = normTurns(rrCfgOld.MagnetSensor.MagnetOffset);

    // Absolute position (in turns) as reported by CANcoder. In Phoenix 6, this reflects the
    // current MagnetOffset configuration. We normalize to [0,1) for stable math/logs.
    double flAbs = normTurns(flCanCoder.getAbsolutePosition().getValueAsDouble());
    double frAbs = normTurns(frCanCoder.getAbsolutePosition().getValueAsDouble());
    double rlAbs = normTurns(rlCanCoder.getAbsolutePosition().getValueAsDouble());
    double rrAbs = normTurns(rrCanCoder.getAbsolutePosition().getValueAsDouble());

    // 2) Compute the NEW offsets so that "current absolute" becomes zero.
    //    If abs is the current angle (turns), the offset that makes it zero is -abs (mod 1).
    double flNewOffset = normTurns(-flAbs);
    double frNewOffset = normTurns(-frAbs);
    double rlNewOffset = normTurns(-rlAbs);
    double rrNewOffset = normTurns(-rrAbs);

    // 3) Apply (persist) the NEW offsets into FLASH and log StatusCodes
    StatusCode sFL = applyOffsetPersist(flCanCoder, flNewOffset);
    StatusCode sFR = applyOffsetPersist(frCanCoder, frNewOffset);
    StatusCode sRL = applyOffsetPersist(rlCanCoder, rlNewOffset);
    StatusCode sRR = applyOffsetPersist(rrCanCoder, rrNewOffset);

    // 4) Read back after write to verify persistence
    CANcoderConfiguration flCfgNew = new CANcoderConfiguration();
    CANcoderConfiguration frCfgNew = new CANcoderConfiguration();
    CANcoderConfiguration rlCfgNew = new CANcoderConfiguration();
    CANcoderConfiguration rrCfgNew = new CANcoderConfiguration();

    flCanCoder.getConfigurator().refresh(flCfgNew);
    frCanCoder.getConfigurator().refresh(frCfgNew);
    rlCanCoder.getConfigurator().refresh(rlCfgNew);
    rrCanCoder.getConfigurator().refresh(rrCfgNew);

    double flReadBack = normTurns(flCfgNew.MagnetSensor.MagnetOffset);
    double frReadBack = normTurns(frCfgNew.MagnetSensor.MagnetOffset);
    double rlReadBack = normTurns(rlCfgNew.MagnetSensor.MagnetOffset);
    double rrReadBack = normTurns(rrCfgNew.MagnetSensor.MagnetOffset);

    // 5) Log everything: old offset, absolute at time of zero, new offset, status, readback, and deltas
    logModule("FL", flAbs, flOldOffset, flNewOffset, flReadBack, sFL);
    logModule("FR", frAbs, frOldOffset, frNewOffset, frReadBack, sFR);
    logModule("RL", rlAbs, rlOldOffset, rlNewOffset, rlReadBack, sRL);
    logModule("RR", rrAbs, rrOldOffset, rrNewOffset, rrReadBack, sRR);

    // 6) Optional sanity check: if any read-back differs from requested by more than epsilon, warn.
    verifyWritten("FL", flNewOffset, flReadBack);
    verifyWritten("FR", frNewOffset, frReadBack);
    verifyWritten("RL", rlNewOffset, rlReadBack);
    verifyWritten("RR", rrNewOffset, rrReadBack);

    System.out.println("[SwerveCal] === Done. Offsets written to CANcoder FLASH. Reboot recommended. ===");
    done = true;
  }

  @Override
  public boolean isFinished() { return done; }

  /**
   * Normalize a value in turns to the range [0, 1).
   * This "wraps" any negative or >1 values into a canonical range.
   *
   * Examples:
   *   normTurns( 1.25) -> 0.25
   *   normTurns(-0.25) -> 0.75
   */
  private static double normTurns(double x) {
    double y = x % 1.0;         // keep only the fractional part (turns)
    if (y < 0) y += 1.0;        // wrap negatives into [0,1)
    return y;
  }

  /**
   * Apply (persist) a MagnetOffset to the device FLASH.
   * The refresh() ensures we start from the existing config and only change MagnetOffset.
   */
  private static StatusCode applyOffsetPersist(CANcoder encoder, double offsetTurns) {
    CANcoderConfiguration cfg = new CANcoderConfiguration();
    encoder.getConfigurator().refresh(cfg);              // read current config from FLASH
    cfg.MagnetSensor.MagnetOffset = offsetTurns;         // set new offset (in turns)
    return encoder.getConfigurator().apply(cfg);         // write & persist to FLASH
  }

  /** Log one module's calibration details in a clear, comparable format. */
  private static void logModule(
      String name,
      double absNow,
      double oldOffset,
      double newOffset,
      double readBack,
      StatusCode status) {

    double deltaOldToNew = minAngleDiffTurns(oldOffset, newOffset);

    System.out.printf(
        "[SwerveCal] %s absNow=%.5f  oldOffset=%.5f  newOffset=%.5f  readBack=%.5f  Δ(old→new)=%.5f  (%s)%n",
        name, absNow, oldOffset, newOffset, readBack, deltaOldToNew, status);
  }

  /**
   * Verify the value persisted by comparing read-back to the requested value.
   * Prints a warning if outside tolerance.
   */
  private static void verifyWritten(String name, double requested, double readBack) {
    double diff = Math.abs(wrapSymmetricTurns(readBack - requested));
    if (diff > kWriteVerifyEps) {
      System.out.printf("[SwerveCal][WARN] %s read-back mismatch: requested=%.6f readBack=%.6f diff=%.6f (turns)%n",
          name, requested, readBack, diff);
    } else {
      System.out.printf("[SwerveCal] %s persisted OK (|diff|=%.6f ≤ %.6f).%n", name, diff, kWriteVerifyEps);
    }
  }

  /**
   * Returns the minimal signed angle difference between a and b (in turns),
   * folded into (-0.5, +0.5]. Useful for comparing two angles modulo 1 turn.
   */
  private static double minAngleDiffTurns(double a, double b) {
    return wrapSymmetricTurns(b - a);
  }

  /**
   * Wraps an angle in turns to the symmetric range (-0.5, +0.5].
   * Example: 0.75 turns -> -0.25 turns (i.e., -90 degrees).
   */
  private static double wrapSymmetricTurns(double x) {
    double y = x % 1.0;
    if (y <= -0.5) y += 1.0;
    if (y >   0.5) y -= 1.0;
    return y;
  }
}

