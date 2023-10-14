package frc.robot.pathing;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.lib.math.CubicBezier;

class Coordinate {
    public double x, y;

    @JsonCreator
    public Coordinate(@JsonProperty("x") double x, @JsonProperty("y") double y) {
        this.x = x;
        this.y = y;
    }

    public Coordinate(Translation2d translation) {
        this(translation.getX(), translation.getY());
    }

    public Translation2d getTranslation() {
        return new Translation2d(this.x, this.y);
    }
}

public class PlannedPath {
    protected List<Waypoint> waypoints;
    protected List<Marker> markers;

    private static class Attributes {
        public List<Waypoint> waypoints;
        public List<Marker> markers;

        @JsonCreator
        public Attributes(@JsonProperty("waypoints") List<Waypoint> waypoints,
                @JsonProperty("markers") List<Marker> markers) {
            this.waypoints = waypoints;
            this.markers = markers;
        }
    }

    public static class Marker {
        private Translation2d location;

        public Translation2d getLocation() {
            return this.location;
        }
    }

    @JsonIgnoreProperties({"isReversal", "velOverride", "isLocked", "isStopPoint", "stopEvent"})
    public static class Waypoint {
        Coordinate anchorPoint, prevControl, nextControl;
        CubicBezier curve;
        double holonomicAngle;

        @JsonCreator
        Waypoint(@JsonProperty("anchorPoint") Coordinate anchorPoint,
                @JsonProperty("prevControl") Coordinate prevControl, @JsonProperty("nextControl") Coordinate nextControl,
                @JsonProperty("holonomicAngle") double holonomicAngle) {
            this.anchorPoint = anchorPoint;
            this.prevControl = prevControl;
            this.nextControl = nextControl;
            this.holonomicAngle = holonomicAngle;
        }

        public CubicBezier getCurve() {
            return this.curve;
        }

        public double getHolonomicAngle() {
            return holonomicAngle;
        }

        public Translation2d getLocation() {
            return this.anchorPoint.getTranslation();
        }
    }

    public PlannedPath(String pathName, File attributes) throws IOException {
        String attributeFilePath = attributes.getAbsolutePath();

        System.out.format("Pathloader: Loading path \"%s\" from \"%s\"...\n",
                pathName, attributeFilePath);

        ObjectMapper mapper = new ObjectMapper();

        Attributes parsedAttributes = mapper.readValue(attributes, Attributes.class);
        this.markers = parsedAttributes.markers;
        this.waypoints = parsedAttributes.waypoints;

        List<Translation2d> splinePoints = new ArrayList<>();
        for (Waypoint waypoint : parsedAttributes.waypoints) {
            if (waypoint.prevControl != null) {
                splinePoints.add(waypoint.prevControl.getTranslation());
            }
            splinePoints.add(waypoint.anchorPoint.getTranslation());
            if (waypoint.nextControl != null) {
                splinePoints.add(waypoint.nextControl.getTranslation());
            }
        }

        for (int j = 1, i = 3; i < splinePoints.size(); j++, i += 3) {
            Translation2d curvePoints[] = {
                    splinePoints.get(i - 3),
                    splinePoints.get(i - 2),
                    splinePoints.get(i - 1),
                    splinePoints.get(i),
            };

            this.waypoints.get(j).curve = new CubicBezier(curvePoints);
        }

        System.out.format("Pathloader: Loaded path \"%s\" from \"%s\"\n", pathName, attributeFilePath);
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

    public List<Marker> getPathMarkers() {
        return this.markers;
    }

    public List<Waypoint> getPathWaypoints() {
        return this.waypoints;
    }
}
