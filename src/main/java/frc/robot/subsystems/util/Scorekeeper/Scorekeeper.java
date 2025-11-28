package frc.robot.subsystems.util.Scorekeeper;

/**
 * Scorekeeper is a tool to calculate the most efficient next step(s) for optimal scoring.
 *
 * <p>Currently "built" for 2025
 */
public class Scorekeeper {
    /** These objects track the gamepieces' position/amount in one place */
    public class pieceTrackers {
        /**
         * Reef (except for L1)
         *
         * @param first index: level (0 = L4, 1 = L3, 2 = L2)
         * @param second index: peg number (0-7 clockwise from left one on the barge side of the
         *     reef)
         */
        boolean[][] reef = new boolean[3][8];

        /** Counts the coral on each side, starting from the closest to the barge */
        public int[] reefL1 = new int[6]; // Counts number per side

        // Algae counters for both net and processor
        public int algaeNet = 0;
        public int algaeProc = 0;

        /**
         * This tracks how many algae are in the original position. First 6 for reef (same order as
         * the coral) and other 3 for ground (left to right)
         */
        public boolean[] algaeUntouched = new boolean[9];
    }

    /** This contains anything that tracks the robot's position or state. */
    public class robotTrackers {
        public boolean hasCoral = false;
        public boolean hasAlgae = false;
    }
}
