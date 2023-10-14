package frc.robot.commands;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.pathing.PlannedPath;
import frc.robot.subsystems.drive.DriveTrainSubsystem;

public class DrivePath extends CommandBase {
    protected DriveTrainSubsystem driveTrain;

    private PlannedPath path;
    private int currentCurve = 0, numWaypoints = -1;

    private DriveLinear linearCommand;
    private DriveSubPath subPathCommand;

    public DrivePath(PlannedPath path, DriveTrainSubsystem driveTrain) {
        this.path = path;
        this.driveTrain = driveTrain;
    }

    private void initialDriveToSart(PlannedPath.Waypoint waypoint, Pose2d currentPose) {
        if (this.linearCommand == null) {
            this.linearCommand = new DriveLinear(0, waypoint.getLocation(),
                    Constants.AutoConstants.minimumInitialPathDistance, this.driveTrain);
            this.linearCommand.schedule();
        }

        if (this.linearCommand.isFinished()) {
            this.currentCurve += 1;
            return;
        }
    }

    private void driveToWaypoint(PlannedPath.Waypoint waypoint, Pose2d currentPose, double endingVelocity) {
        if (this.subPathCommand == null) {
            this.subPathCommand = new DriveSubPath(waypoint.getCurve(), Constants.AutoConstants.pathingTolerance,
                    endingVelocity, this.driveTrain);
            this.subPathCommand.schedule();
        }

        if (this.subPathCommand.isFinished()) {
            this.currentCurve += 1;
            this.subPathCommand = null;
            return;
        }
    }

    @Override
    public boolean isFinished() {
        return (this.numWaypoints != -1) && (this.currentCurve >= this.numWaypoints);
    }

    @Override
    public void execute() {
        Logger logger = Logger.getInstance();
        List<PlannedPath.Waypoint> waypoints = this.path.getPathWaypoints();
        Pose2d currentPose = this.driveTrain.getPose();

        this.numWaypoints = waypoints.size();

        logger.recordOutput("/Pathing/currentCurve", this.currentCurve);
        logger.recordOutput("/Pathing/numWaypoints", this.numWaypoints);

        // Starting waypoint
        if (this.currentCurve == 0) {
            this.initialDriveToSart(waypoints.get(0), currentPose);
        }
        // Intermediate waypoint
        else if (this.currentCurve < waypoints.size() - 1){
            this.driveToWaypoint(waypoints.get(this.currentCurve), currentPose,
                    Constants.AutoConstants.MaxSpeedMetersPerSecond);
        }
        // Ending waypoint
        else if (this.currentCurve == waypoints.size() - 1) {
            this.driveToWaypoint(waypoints.get(this.currentCurve), currentPose, 0);
        }
    }
}
