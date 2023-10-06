package frc.robot.pathing;

import edu.wpi.first.math.geometry.Translation2d;

public class PathWaypoint {
    private Translation2d anchorPoint, prevControl, nextControl;

    public Translation2d getAnchor() {
        return this.anchorPoint;
    }

    public Translation2d getPrevControl() {
        return this.prevControl;
    }
    public Translation2d getNextControl() {
        return this.nextControl;
    }
}
