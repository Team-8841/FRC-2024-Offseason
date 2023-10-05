package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.pathing.PathInstant;
import frc.robot.pathing.PathWaypoint;
import frc.robot.pathing.PlannedPath;
import frc.robot.sensors.imu.IMU;
import frc.robot.subsystems.drive.DriveTrainSubsystem;

public class PathingCommand extends CommandBase {
    private PlannedPath path;
    private List<PathInstant> pathInstants;
    private List<PathWaypoint> pathWaypoints;

    private boolean mustDriveToStart = false;

    private DriveTrainSubsystem driveTrain;
    private IMU imu;

    public PathingCommand(PlannedPath path, DriveTrainSubsystem driveTrain, IMU imu) {
        this.path = path;
        this.pathInstants = this.path.getInstants();
        this.pathWaypoints = this.path.getPathWaypoints();

        this.driveTrain = driveTrain;
        this.imu = imu;

        this.addRequirements(driveTrain);

        Translation2d startingTrans = this.pathInstants.get(0).pose.getTranslation();
        Translation2d currentTrans = this.driveTrain.getPose().getTranslation();

        if (startingTrans.getDistance(currentTrans) > Constants.minimumInitialAnchorDistance) {
            this.mustDriveToStart = true;
        }
    }

    @Override
    public void execute() {
        Translation2d currentTrans = this.driveTrain.getPose().getTranslation();

        if (this.mustDriveToStart) {
            Translation2d startingTrans = this.pathInstants.get(0).pose.getTranslation();
            PathWaypoint startingWaypoint = this.pathWaypoints.get(0);
            
            

            if (startingTrans.getDistance(currentTrans) > Constants.minimumInitialAnchorDistance) {
                return;
            }

            this.mustDriveToStart = false;
        }
    }
}
