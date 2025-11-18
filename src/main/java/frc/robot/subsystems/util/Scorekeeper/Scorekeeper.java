package frc.robot.subsystems.util.Scorekeeper;

/** Scorekeeper is a tool to calculate the most efficient next step for optimal scoring.
 * <p> Currently "built" for 2025*/
public class Scorekeeper {
    /** Reef (except for L1)
     * @param first index: level (0 = L4, 1 = L3, 2 = L2)
     * @param second index: peg number (0-7 clockwise from left one on the barge side of the reef) */
    boolean[][] reef = new boolean[3][8];    
    /** Counts the coral on each  side, starting from the closest to the barge */
    public int[] reefL1 = new int[6]; // Counts number per side
}
