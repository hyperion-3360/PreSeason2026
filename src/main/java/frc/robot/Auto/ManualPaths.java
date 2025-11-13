package frc.robot.Auto;

public enum ManualPaths {
    PATH1("/path/to/pathplanner/path.pathplanner"),
    PATH2("/path/to/pathplanner/path.pathplanner"),
    PATH3("/path/to/pathplanner/path.pathplanner");

    private String pathName;

    ManualPaths(String pathName) {
        this.pathName = pathName;
    }

    String getString() {
        return pathName;
    }
}
