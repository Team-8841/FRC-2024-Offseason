package frc.robot.pathing;

public class PathWaypoint implements Positioned {
    private Coordinate anchorPoint, prevControl, nextControl;

    public Coordinate getPosition() {
        return this.anchorPoint;
    }
}
