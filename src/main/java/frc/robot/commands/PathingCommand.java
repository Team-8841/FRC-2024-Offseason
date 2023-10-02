package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.imu.IMU;
import frc.robot.subsystems.drive.DriveTrainSubsystem;

public class PathingCommand extends CommandBase {
    private DriveTrainSubsystem driveTrain;
    private IMU imu;

    public PathingCommand(DriveTrainSubsystem driveTrain, IMU imu) {
        this.driveTrain = driveTrain;
        this.imu = imu;

        this.addRequirements(driveTrain);
    }

    @Override
    public void execute() {
    }
}
