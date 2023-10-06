package frc.robot.pathing;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

import com.fasterxml.jackson.annotation.JsonCreator;

import frc.robot.Constants;

@JsonIgnoreProperties({ "velocity", "acceleration", "curvature", "holonomicRotation", "angularVelocity",
        "holonomicAngularVelocity" })
public class PathInstant {
    public double time;
    public Pose2d pose;

    private static class PathRotation {
        public double radians;
    }

    private static class PathTranslation {
        public double x, y;
    }

    private static class PathPose {
        public PathTranslation translation;
        public PathRotation rotation;
    }

    @JsonCreator
    public PathInstant(@JsonProperty("time") double time, @JsonProperty("pose") PathPose pose) {
        this.time = time;
        this.pose = new Pose2d(pose.translation.x, pose.translation.y, Rotation2d.fromRadians(pose.rotation.radians));
    }

    public static int getInstantFromPosition(List<PathInstant> instants, double t, Translation2d position) {
        for (int i = 0; i < instants.size(); i++) {
            PathInstant instant = instants.get(i);

            if (instant.time < t) {
                continue; // skip
            }

            if (instant.pose.getTranslation().getDistance(position) <= Constants.pathingEpsilon) {
                return i;
            }
        }

        return -1;
    }

    // TODO: Maybe replace this with a binary search?
    public static int getInstantAtT(List<PathInstant> instants, double t) {
        if (t >= 1) {
            return instants.size() - 1;
        }

        for (int i = 1; i < instants.size(); i++) {
            if (instants.get(i).time > t) {
                return i - 1;
            }
        }

        // Shouldn't ever happen
        return instants.size() - 1;
    }
}