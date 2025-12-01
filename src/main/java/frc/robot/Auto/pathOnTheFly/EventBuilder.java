package frc.robot.Auto.pathOnTheFly;

import com.pathplanner.lib.path.EventMarker;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;

public class EventBuilder {

    public record EventMarkerSpecs(String commandName, Command command, double... positions) {}

    /**
     * Creates an event marker for every commands
     *
     * @param commandToExecuteOnPath The command to be registered
     * @return A list of all commands that have been transfored into event markers
     * @throws Exception If the zoning values aren't well inputed
     */
    public List<EventMarker> generateEventMarkers(List<EventMarkerSpecs> commandToExecuteOnPath)
            throws Exception {

        List<EventMarker> eventList = new ArrayList<>();

        for (EventMarkerSpecs specs : commandToExecuteOnPath) {

            if (specs.positions.length == 0) {
                throw new IllegalArgumentException(
                        "Event must specify at least one path position. -1.0 means no zoning.");
            }

            for (double position : specs.positions) {
                if (position > 1.0) {
                    throw new IllegalArgumentException("Event position must be less than 1.0.");
                }
            }

            if (specs.positions.length == 1) {
                eventList.add(
                        new EventMarker(specs.commandName, specs.positions[0], specs.command));
            } else if (specs.positions.length == 2) {
                eventList.add(
                        new EventMarker(
                                specs.commandName,
                                specs.positions[0],
                                specs.positions[1],
                                specs.command));
            } else {
                throw new IllegalArgumentException("An event can't have more than two positions.");
            }
        }

        return eventList;
    }
}
