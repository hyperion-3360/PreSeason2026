package frc.robot;

/**
 * Central hardware profile switch for swerve modules. Flip ACTIVE_SWERVE to target your hardware
 * without touching generated files.
 *
 * <p>NOTE: Fill the SDS_MK4N constants with your real values (IDs, inversions) once your MK4n kit
 * is installed. Gear ratios are set to official SDS MK4n L2 specs.
 */
public final class RobotConfig {
    private RobotConfig() {}

    public enum SwerveProfile {
        WCP_SWERVEX_CTRE,
        SDS_MK4N_L2 // MK4n (newer version), not MK4i
    }

    public enum MotorType {
        KRAKEN_X60, // More powerful, better cooling
        FALCON_500 // Standard TalonFX
    }

    /** Change this single line to switch robot hardware profile. */
    public static final SwerveProfile ACTIVE_SWERVE = SwerveProfile.SDS_MK4N_L2;

    /** Change these to mix and match motor types for drive and steer. */
    public static final MotorType DRIVE_MOTOR = MotorType.FALCON_500;

    public static final MotorType STEER_MOTOR = MotorType.FALCON_500;

    /** CANivore name - update this to match your Phoenix Tuner X configuration. */
    public static final String CANIVORE_NAME = "CANivore_3360";

    // ---- WCP Swerve X (current robot) ----
    private static final class WCP {
        // IDs from your current TunerConstants.java
        static final int FL_DRIVE = 7, FL_STEER = 8, FL_ENC = 16;
        static final int FR_DRIVE = 5, FR_STEER = 6, FR_ENC = 14;
        static final int BL_DRIVE = 1, BL_STEER = 2, BL_ENC = 13;
        static final int BR_DRIVE = 3, BR_STEER = 4, BR_ENC = 15;

        // Inversions as in current project
        static final boolean FL_STEER_INV = true, FL_ENC_INV = false;
        static final boolean FR_STEER_INV = true, FR_ENC_INV = false;
        static final boolean BL_STEER_INV = true, BL_ENC_INV = false;
        static final boolean BR_STEER_INV = true, BR_ENC_INV = false;

        // Kinematics and gearing
        static final double DRIVE_GEAR_RATIO = 7.363636363636365;
        static final double STEER_GEAR_RATIO = 13.371428571428572;
        static final double COUPLE_RATIO = 3.8181818181818183;
        static final double WHEEL_RADIUS_IN = 1.875; // inches
        static final double SPEED_12V_MPS = 4.06; // m/s (feedforward scale)
    }

    // ---- SDS MK4n L2 (new kit) ----
    private static final class MK4N {
        // TODO: Replace placeholders with real IDs when wired
        static final int FL_DRIVE = 1, FL_STEER = 2, FL_ENC = 33;
        static final int FR_DRIVE = 3, FR_STEER = 4, FR_ENC = 32;
        static final int BL_DRIVE = 7, BL_STEER = 8, BL_ENC = 34;
        static final int BR_DRIVE = 5, BR_STEER = 6, BR_ENC = 31;

        // TODO: Confirm/adjust inversions for MK4n once installed
        static final boolean FL_STEER_INV = WCP.FL_STEER_INV, FL_ENC_INV = WCP.FL_ENC_INV;
        static final boolean FR_STEER_INV = WCP.FR_STEER_INV, FR_ENC_INV = WCP.FR_ENC_INV;
        static final boolean BL_STEER_INV = WCP.BL_STEER_INV, BL_ENC_INV = WCP.BL_ENC_INV;
        static final boolean BR_STEER_INV = WCP.BR_STEER_INV, BR_ENC_INV = WCP.BR_ENC_INV;

        // Official SDS MK4n L2 gear ratios (confirmed from SDS spec sheet)
        static final double DRIVE_GEAR_RATIO = 5.357142857142857; // L2 = 5.36:1
        static final double STEER_GEAR_RATIO = 21.428571428571427; // MK4n = 150/7
        static final double COUPLE_RATIO = 3.5555555555555554; // Coupling ratio for MK4n
        static final double WHEEL_RADIUS_IN = 2.0; // 4" wheel -> 2.0" radius
        static final double SPEED_12V_MPS = 4.73; // Theoretical max speed L2 w/ Falcon 500
    }

    // ---- Motor-Specific Current Limits ----
    private static final class KrakenLimits {
        // Kraken X60 can handle more current and has better thermal management
        // "Balanced" config - Good performance while staying safe
        static final double DRIVE_STATOR_LIMIT_A = 80.0; // Thermal protection (continuous)
        static final double DRIVE_SUPPLY_LIMIT_A = 60.0; // Battery protection (continuous)
        static final double DRIVE_SUPPLY_BURST_A = 120.0; // Burst allowance (+40A for 1s)
        static final double DRIVE_SUPPLY_BURST_TIME_S = 1.0; // 1 second burst window
        static final double STEER_STATOR_LIMIT_A = 60.0;
        static final double SLIP_CURRENT_A = 75.0; // Just below stator limit for slip detection
    }

    private static final class FalconLimits {
        // Falcon 500 thermal limits - balanced for competition
        // "Balanced" config - ~30% more burst performance vs conservative
        static final double DRIVE_STATOR_LIMIT_A = 60.0; // Thermal protection (continuous)
        static final double DRIVE_SUPPLY_LIMIT_A = 50.0; // Battery protection (continuous)
        static final double DRIVE_SUPPLY_BURST_A = 80.0; // Burst allowance (+30A for 0.75s)
        static final double DRIVE_SUPPLY_BURST_TIME_S = 0.75; // 0.75 second burst window
        static final double STEER_STATOR_LIMIT_A = 40.0;
        static final double SLIP_CURRENT_A = 55.0; // Just below stator limit for slip detection
    }

    // ---- Public helpers used by TunerConstants ----
    public static boolean isMk4n() {
        return ACTIVE_SWERVE == SwerveProfile.SDS_MK4N_L2;
    }

    public static int flDrive() {
        return isMk4n() ? MK4N.FL_DRIVE : WCP.FL_DRIVE;
    }

    public static int flSteer() {
        return isMk4n() ? MK4N.FL_STEER : WCP.FL_STEER;
    }

    public static int flEnc() {
        return isMk4n() ? MK4N.FL_ENC : WCP.FL_ENC;
    }

    public static int frDrive() {
        return isMk4n() ? MK4N.FR_DRIVE : WCP.FR_DRIVE;
    }

    public static int frSteer() {
        return isMk4n() ? MK4N.FR_STEER : WCP.FR_STEER;
    }

    public static int frEnc() {
        return isMk4n() ? MK4N.FR_ENC : WCP.FR_ENC;
    }

    public static int blDrive() {
        return isMk4n() ? MK4N.BL_DRIVE : WCP.BL_DRIVE;
    }

    public static int blSteer() {
        return isMk4n() ? MK4N.BL_STEER : WCP.BL_STEER;
    }

    public static int blEnc() {
        return isMk4n() ? MK4N.BL_ENC : WCP.BL_ENC;
    }

    public static int brDrive() {
        return isMk4n() ? MK4N.BR_DRIVE : WCP.BR_DRIVE;
    }

    public static int brSteer() {
        return isMk4n() ? MK4N.BR_STEER : WCP.BR_STEER;
    }

    public static int brEnc() {
        return isMk4n() ? MK4N.BR_ENC : WCP.BR_ENC;
    }

    public static boolean flSteerInv() {
        return isMk4n() ? MK4N.FL_STEER_INV : WCP.FL_STEER_INV;
    }

    public static boolean flEncInv() {
        return isMk4n() ? MK4N.FL_ENC_INV : WCP.FL_ENC_INV;
    }

    public static boolean frSteerInv() {
        return isMk4n() ? MK4N.FR_STEER_INV : WCP.FR_STEER_INV;
    }

    public static boolean frEncInv() {
        return isMk4n() ? MK4N.FR_ENC_INV : WCP.FR_ENC_INV;
    }

    public static boolean blSteerInv() {
        return isMk4n() ? MK4N.BL_STEER_INV : WCP.BL_STEER_INV;
    }

    public static boolean blEncInv() {
        return isMk4n() ? MK4N.BL_ENC_INV : WCP.BL_ENC_INV;
    }

    public static boolean brSteerInv() {
        return isMk4n() ? MK4N.BR_STEER_INV : WCP.BR_STEER_INV;
    }

    public static boolean brEncInv() {
        return isMk4n() ? MK4N.BR_ENC_INV : WCP.BR_ENC_INV;
    }

    public static double driveGearRatio() {
        return isMk4n() ? MK4N.DRIVE_GEAR_RATIO : WCP.DRIVE_GEAR_RATIO;
    }

    public static double steerGearRatio() {
        return isMk4n() ? MK4N.STEER_GEAR_RATIO : WCP.STEER_GEAR_RATIO;
    }

    public static double coupleRatio() {
        return isMk4n() ? MK4N.COUPLE_RATIO : WCP.COUPLE_RATIO;
    }

    public static double wheelRadiusIn() {
        return isMk4n() ? MK4N.WHEEL_RADIUS_IN : WCP.WHEEL_RADIUS_IN;
    }

    public static double speedAt12V() {
        return isMk4n() ? MK4N.SPEED_12V_MPS : WCP.SPEED_12V_MPS;
    }

    // ---- Current Limit Getters (based on motor type) ----
    private static boolean isDriveKraken() {
        return DRIVE_MOTOR == MotorType.KRAKEN_X60;
    }

    private static boolean isSteerKraken() {
        return STEER_MOTOR == MotorType.KRAKEN_X60;
    }

    public static double driveStatorLimitAmps() {
        return isDriveKraken()
                ? KrakenLimits.DRIVE_STATOR_LIMIT_A
                : FalconLimits.DRIVE_STATOR_LIMIT_A;
    }

    public static double driveSupplyLimitAmps() {
        return isDriveKraken()
                ? KrakenLimits.DRIVE_SUPPLY_LIMIT_A
                : FalconLimits.DRIVE_SUPPLY_LIMIT_A;
    }

    public static double driveSupplyBurstAmps() {
        return isDriveKraken()
                ? KrakenLimits.DRIVE_SUPPLY_BURST_A
                : FalconLimits.DRIVE_SUPPLY_BURST_A;
    }

    public static double driveSupplyBurstTimeSeconds() {
        return isDriveKraken()
                ? KrakenLimits.DRIVE_SUPPLY_BURST_TIME_S
                : FalconLimits.DRIVE_SUPPLY_BURST_TIME_S;
    }

    public static double steerStatorLimitAmps() {
        return isSteerKraken()
                ? KrakenLimits.STEER_STATOR_LIMIT_A
                : FalconLimits.STEER_STATOR_LIMIT_A;
    }

    public static double driveSlipCurrentAmps() {
        return isDriveKraken() ? KrakenLimits.SLIP_CURRENT_A : FalconLimits.SLIP_CURRENT_A;
    }

    /**
     * Returns the DCMotor model for the configured drive motor type.
     *
     * @param numMotors Number of motors per module (typically 1)
     * @return DCMotor instance (Kraken X60 or Falcon 500)
     */
    public static edu.wpi.first.math.system.plant.DCMotor getDriveMotor(int numMotors) {
        return isDriveKraken()
                ? edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(numMotors)
                : edu.wpi.first.math.system.plant.DCMotor.getFalcon500(numMotors);
    }
}
