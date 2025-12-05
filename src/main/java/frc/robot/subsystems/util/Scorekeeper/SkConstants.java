package frc.robot.subsystems.util.Scorekeeper;

/** Currently has scoring info for 2025 */
public final class SkConstants {
    public static final class teleopScoring {
        // coral scoring
        public static final int coralL4 = 5;
        public static final int coralL3 = 4;
        public static final int coralL2 = 3;
        public static final int coralL1 = 2;
        // algae scoring
        public static final int algaeNet = 4;
        public static final int algaeProc = 2;
        // climb (or not) scoring
        public static final int bargeClimb = 12;
        public static final int bargePark = 2;
    }

    public static final class autoScoring {
        // coral scoring
        public static final int coralL4 = 7;
        public static final int coralL3 = 6;
        public static final int coralL2 = 4;
        public static final int coralL1 = 3;
        // algae scoring
        public static final int algaeNet = 4;
        public static final int algaeProc = 2;
        // leave points
        public static final int leave = 3;
    }

    public static final class driveConstants {
        /* These are more for coordinate-based positions
        // swerve speed stats
        public static final double maxAccel = 6.0; // m/s²
        public static final double maxSpeed = 4.5; // m/s

        // slowdown values (keep these in the 1.0 - 1.5 range if possible)
        // this is here to slow down the calculated time to account for stuff
        public static final double teleopTimeMult = 1.3; //placeholder
        public static final double autoTimeMult = 1.1; //placeholder
        */

        // climb time (placeholder)
        public static final double climbTime = 8.0; // seconds

        /*
        // feeding station delay (placeholder)
        public static final double feedDelay = 0.2; // seconds
        */
    }

    // might stay unused for a while
    public static final class rankingPoints {
        public static final int rpWin = 3;
        public static final int rpTie = 1;
        public static final int rpBarge = 1;
        public static final int rpCoral = 1;
        public static final int rpAuto = 1;
    }
}
