package frc.robot.pathing;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;

class PathAttributes {
    public List<PathWaypoint> waypoints;
    public List<PathMarker> markers;
}

class Coordinate {
    public double x, y;

    public Coordinate(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Coordinate(Translation2d translation) {
        this(translation.getX(), translation.getY());
    }

    public Translation2d getTranslatin() {
        return new Translation2d(this.x, this.y);
    }
}

public class PlannedPath {
    protected List<PathInstant> instants;
    protected List<PathWaypoint> waypoints;
    protected List<PathMarker> markers;

    public PlannedPath(String pathName, File attributes) throws IOException {
        String attributeFilePath = attributes.getAbsolutePath();

        System.out.format("Pathloader: Loading path \"%s\" from \"%s\"...\n",
                pathName, attributeFilePath);

        ObjectMapper mapper = new ObjectMapper();
        
        PathAttributes parsedAttributes = mapper.readValue(attributes, PathAttributes.class);
        this.waypoints = parsedAttributes.waypoints;
        this.markers = parsedAttributes.markers;

        System.out.format("Pathloader: Loaded path \"%s\" from \"%s\"\n"
                + "Pathloader: num of waypoints: %d\n"
                + "Pathloader: num of markers: %d\n",
                pathName, attributeFilePath,
                this.waypoints.size(), this.markers.size());
        
        this.calculateInstants();
        this.normalizeInstants(this.instants.get(this.instants.size() - 1).time);
    }

    private void calculateInstants() {
        for (int i = 1; i < this.waypoints.size(); i++) {
            PathWaypoint lasWaypoint = this.waypoints.get(i - 1);
            PathWaypoint curWaypoint = this.waypoints.get(i);

            for (double t = 0; t <= 1; t += 1.0 / (Constants.pathInstantCount - 1)) {

            }
        }
    }

    public static PlannedPath fetchPath(String pathName) {
        Path deployPath = Filesystem.getDeployDirectory().toPath();
        Path attributesPath = deployPath.resolve("pathplanner").resolve(pathName + ".path");

        try {
            return new PlannedPath(pathName, attributesPath.toFile());
        } catch (IOException e) {
            System.err.format("Pathloader: Encountered error while reading path \"%s\": %s\n", pathName, e.toString());

            return null;
        }
    }

    private void normalizeInstants(double factor) {
        for (int i = 0; i < this.instants.size(); i++) {
            this.instants.get(i).time /= factor;
        }
    }

    public List<PathInstant> getInstants() {
        return this.instants;
    }

    public List<PathWaypoint> getPathWaypoints() {
        return this.waypoints;
    }

    public List<PathMarker> getPathMarkers() {
        return this.markers;
    }

}
