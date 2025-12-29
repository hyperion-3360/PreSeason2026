package frc.robot;

/**
 * Central hardware profile switch for swerve modules. Flip ACTIVE_SWERVE to target your hardware
 * without touching generated files.
 *
 * <p>NOTE: Fill the SDS_MK4I constants with your real values (IDs, inversions, ratios) once your
 * MK4i kit is installed. For now, they mirror the WCP values so code compiles.
 */
public final class RobotConfig {
    private RobotConfig() {}

    public enum SwerveProfile {
        WCP_SWERVEX_CTRE,
        SDS_MK4I_L2
    }

    public enum MotorType {
        KRAKEN_X60, // More powerful, better cooling
        FALCON_500 // Standard TalonFX
    }

    /** Change this single line to switch robot hardware profile. */
    public static final SwerveProfile ACTIVE_SWERVE = SwerveProfile.SDS_MK4I_L2;

    /** Change these to mix and match motor types for drive and steer. */
    public static final MotorType DRIVE_MOTOR = MotorType.FALCON_500;

    public static final MotorType STEER_MOTOR = MotorType.FALCON_500;

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

    // ---- SDS MK4i L2 (new kit) ----
    private static final class MK4I {
        // TODO: Replace placeholders with real IDs when wired
        static final int FL_DRIVE = 1, FL_STEER = 2, FL_ENC = 33;
        static final int FR_DRIVE = 3, FR_STEER = 4, FR_ENC = 32;
        static final int BL_DRIVE = 7, BL_STEER = 8, BL_ENC = 34;
        static final int BR_DRIVE = 5, BR_STEER = 6, BR_ENC = 31;

        // TODO: Confirm/adjust inversions for MK4i once installed
        static final boolean FL_STEER_INV = WCP.FL_STEER_INV, FL_ENC_INV = WCP.FL_ENC_INV;
        static final boolean FR_STEER_INV = WCP.FR_STEER_INV, FR_ENC_INV = WCP.FR_ENC_INV;
        static final boolean BL_STEER_INV = WCP.BL_STEER_INV, BL_ENC_INV = WCP.BL_ENC_INV;
        static final boolean BR_STEER_INV = WCP.BR_STEER_INV, BR_ENC_INV = WCP.BR_ENC_INV;

        // TODO: Set official MK4i L2 ratios and wheel radius (inches) once confirmed
        static final double DRIVE_GEAR_RATIO = 5.902777777777778; // placeholder
        static final double STEER_GEAR_RATIO = 18.75; // placeholder
        static final double COUPLE_RATIO = 3.125; // placeholder
        static final double WHEEL_RADIUS_IN = 2.0; // 4" wheel -> 2.0" radius
        static final double SPEED_12V_MPS = WCP.SPEED_12V_MPS; // placeholder until SysId
    }

    // ---- Motor-Specific Current Limits ----
    private static final class KrakenLimits {
        // Kraken X60 can handle more current and has better thermal management
        static final double DRIVE_STATOR_LIMIT_A = 80.0;
        static final double DRIVE_SUPPLY_LIMIT_A = 60.0;
        static final double DRIVE_SUPPLY_BURST_A = 80.0;
        static final double DRIVE_SUPPLY_BURST_TIME_S = 0.5;
        static final double STEER_STATOR_LIMIT_A = 60.0;
    }

    private static final class FalconLimits {
        // Falcon 500 thermal limits - more conservative than Kraken
        static final double DRIVE_STATOR_LIMIT_A = 60.0;
        static final double DRIVE_SUPPLY_LIMIT_A = 50.0;
        static final double DRIVE_SUPPLY_BURST_A = 65.0;
        static final double DRIVE_SUPPLY_BURST_TIME_S = 0.5;
        static final double STEER_STATOR_LIMIT_A = 40.0;
    }

    // ---- Public helpers used by TunerConstants ----
    public static boolean isMk4i() {
        return ACTIVE_SWERVE == SwerveProfile.SDS_MK4I_L2;
    }

    public static int flDrive() {
        return isMk4i() ? MK4I.FL_DRIVE : WCP.FL_DRIVE;
    }

    public static int flSteer() {
        return isMk4i() ? MK4I.FL_STEER : WCP.FL_STEER;
    }

    public static int flEnc() {
        return isMk4i() ? MK4I.FL_ENC : WCP.FL_ENC;
    }

    public static int frDrive() {
        return isMk4i() ? MK4I.FR_DRIVE : WCP.FR_DRIVE;
    }

    public static int frSteer() {
        return isMk4i() ? MK4I.FR_STEER : WCP.FR_STEER;
    }

    public static int frEnc() {
        return isMk4i() ? MK4I.FR_ENC : WCP.FR_ENC;
    }

    public static int blDrive() {
        return isMk4i() ? MK4I.BL_DRIVE : WCP.BL_DRIVE;
    }

    public static int blSteer() {
        return isMk4i() ? MK4I.BL_STEER : WCP.BL_STEER;
    }

    public static int blEnc() {
        return isMk4i() ? MK4I.BL_ENC : WCP.BL_ENC;
    }

    public static int brDrive() {
        return isMk4i() ? MK4I.BR_DRIVE : WCP.BR_DRIVE;
    }

    public static int brSteer() {
        return isMk4i() ? MK4I.BR_STEER : WCP.BR_STEER;
    }

    public static int brEnc() {
        return isMk4i() ? MK4I.BR_ENC : WCP.BR_ENC;
    }

    public static boolean flSteerInv() {
        return isMk4i() ? MK4I.FL_STEER_INV : WCP.FL_STEER_INV;
    }

    public static boolean flEncInv() {
        return isMk4i() ? MK4I.FL_ENC_INV : WCP.FL_ENC_INV;
    }

    public static boolean frSteerInv() {
        return isMk4i() ? MK4I.FR_STEER_INV : WCP.FR_STEER_INV;
    }

    public static boolean frEncInv() {
        return isMk4i() ? MK4I.FR_ENC_INV : WCP.FR_ENC_INV;
    }

    public static boolean blSteerInv() {
        return isMk4i() ? MK4I.BL_STEER_INV : WCP.BL_STEER_INV;
    }

    public static boolean blEncInv() {
        return isMk4i() ? MK4I.BL_ENC_INV : WCP.BL_ENC_INV;
    }

    public static boolean brSteerInv() {
        return isMk4i() ? MK4I.BR_STEER_INV : WCP.BR_STEER_INV;
    }

    public static boolean brEncInv() {
        return isMk4i() ? MK4I.BR_ENC_INV : WCP.BR_ENC_INV;
    }

    public static double driveGearRatio() {
        return isMk4i() ? MK4I.DRIVE_GEAR_RATIO : WCP.DRIVE_GEAR_RATIO;
    }

    public static double steerGearRatio() {
        return isMk4i() ? MK4I.STEER_GEAR_RATIO : WCP.STEER_GEAR_RATIO;
    }

    public static double coupleRatio() {
        return isMk4i() ? MK4I.COUPLE_RATIO : WCP.COUPLE_RATIO;
    }

    public static double wheelRadiusIn() {
        return isMk4i() ? MK4I.WHEEL_RADIUS_IN : WCP.WHEEL_RADIUS_IN;
    }

    public static double speedAt12V() {
        return isMk4i() ? MK4I.SPEED_12V_MPS : WCP.SPEED_12V_MPS;
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
}
