package frc.robot.subsystems.util.Scorekeeper;

/** Scorekeeper is a tool to calculate the most efficient next step for optimal scoring.
 * <p> Currently "built" for 2025*/
public class Scorekeeper {
    /** These objects track the gamepieces' position/amount in one place */
    public class pieceTrackers {
    /** Reef (except for L1)
     * @param first index: level (0 = L4, 1 = L3, 2 = L2)
     * @param second index: peg number (0-7 clockwise from left one on the barge side of the reef) */
    boolean[][] reef = new boolean[3][8];    
    /** Counts the coral on each side, starting from the closest to the barge */
    public int[] reefL1 = new int[6]; // Counts number per side

    // Algae counters for both net and processor
    public int algaeNet = 0;
    public int algaeProc = 0;

    /** This tracks how many algae are in the original position (6 reef and 3 ground) */
    public boolean[] algaeUntouched = new boolean[9];
    
    }
}
