package frc.robot.pathing;

public class PathWaypoint implements Positioned {
    private Coordinate anchorPoint, prevControl, nextControl;

    public Coordinate getAnchor() {
        return this.anchorPoint;
    }

    public Coordinate getPrevControl() {
        return this.prevControl;
    }
    public Coordinate getNextControl() {
        return this.nextControl;
    }

    public Coordinate getPosition() {
        return this.anchorPoint;
    }
}
