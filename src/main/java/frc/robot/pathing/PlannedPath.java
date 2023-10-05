package frc.robot.pathing;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

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

    public Coordinate(Pose2d pose) {
        this(pose.getX(), pose.getY());
    }

    public double distanceSquared(double x, double y) {
        return (x - this.x) * (x - this.x) + (y - this.y) * (y - this.y);
    }

    public double distanceSquared(Coordinate other) {
        return this.distanceSquared(other.x, other.y);
    }

    public Pose2d getPose() {
        return new Pose2d(this.x, this.y, new Rotation2d());
    }
}

interface Positioned {
    public Coordinate getPosition();
}

public class PlannedPath {
    protected List<PathInstant> instants;
    protected List<PathWaypoint> waypoints;
    protected List<PathMarker> markers;

    public PlannedPath(File precomputedData, File attributes) throws IOException {
        String precomputedFilePath = precomputedData.getAbsolutePath();
        String attributeFilePath = attributes.getAbsolutePath();

        System.out.format("Pathloader: Loading path from \"%s\" and \"%s\"...\n", precomputedFilePath,
                attributeFilePath);

        ObjectMapper mapper = new ObjectMapper();

        this.instants = mapper.readValue(precomputedData, new TypeReference<List<PathInstant>>() {
        });
        
        PathAttributes parsedAttributes = mapper.readValue(attributes, PathAttributes.class);
        this.waypoints = parsedAttributes.waypoints;
        this.markers = parsedAttributes.markers;

        System.out.format("Pathloader: Loaded path from \"%s\" and \"%s\"\n"
                + "Pathloader: num of instants: %d\n"
                + "Pathloader: num of waypoints: %d\n"
                + "Pathloader: num of markers: %d\n",
                precomputedFilePath, attributeFilePath,
                this.instants.size(), this.waypoints.size(), this.markers.size());
        
        this.normalizeInstants(this.instants.get(this.instants.size() - 1).time);
    }

    public static PlannedPath fetchPath(String pathName) {
        Path deployPath = Filesystem.getDeployDirectory().toPath();
        Path precomputedDataPath = deployPath.resolve("pathplanner/generatedJSON").resolve(pathName + ".wpilib.json");
        Path attributesPath = deployPath.resolve("pathplanner").resolve(pathName + ".path");

        try {
            return new PlannedPath(precomputedDataPath.toFile(), attributesPath.toFile());
        } catch (IOException e) {
            Logger.getInstance().recordOutput("/Errors", e.toString());
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
