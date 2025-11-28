package frc.robot.Auto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoChooser {

    public enum Mode {
        COUCHETARD("Circle K"),
        AVANCE("Line"),
        SOURI("Smile"),
        LITERALLYNOTHING("rien pentoute");

        private String m_path;

        private Mode(String path) {
            m_path = path;
        }

        public String toString() {
            return m_path;
        }
    }

    // public static Command followPath(Mode automode) {
    //   PathPlannerPath path = PathPlannerPath.fromPathFile("Test");

    //   return AutoBuilder.followPath(path);
    // }

    private static SendableChooser<Mode> autoChooser = new SendableChooser<>();

    public static void setShuffleboardOptions() {
        autoChooser.setDefaultOption("score 4 notes close", Mode.LITERALLYNOTHING);

        autoChooser.addOption("Full circle around the reaf", Mode.COUCHETARD);
        autoChooser.addOption("Straight Line", Mode.AVANCE);
        autoChooser.addOption("Smile around tha reaf", Mode.SOURI);
        autoChooser.addOption("do nothing", Mode.LITERALLYNOTHING);

        Shuffleboard.getTab("Autos").add("Auto Mode", autoChooser);

        // Récupérer le mode autonome sélectionné
    }

    public static Mode getSelectedOption() {
        return autoChooser.getSelected();
    }
}
