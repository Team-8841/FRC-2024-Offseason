package frc.robot.pathing;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.fasterxml.jackson.annotation.JsonCreator;

class PathRotation {
    public double radians;
}

class PathTranslation {
    public double x, y;
}

class PathPose {
    public PathTranslation translation;
    public PathRotation rotation;
}

@JsonIgnoreProperties({ "velocity", "acceleration", "curvature", "holonomicRotation", "angularVelocity",
        "holonomicAngularVelocity" })
public class PathInstant implements Positioned {
    public double time;
    public Pose2d pose;

    @JsonCreator
    public PathInstant(@JsonProperty("time") double time, @JsonProperty("pose") PathPose pose) {
        this.time = time;
        this.pose = new Pose2d(pose.translation.x, pose.translation.y, Rotation2d.fromRadians(pose.rotation.radians));
    }

    Coordinate getPosition() {
        return new Coordinate(this.pose);
    }
}