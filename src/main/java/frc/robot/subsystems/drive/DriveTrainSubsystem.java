package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.imu.IMU;

/**
 * Main swerve drive class, interfaces with a hardware IO class.
 */
public class DriveTrainSubsystem extends SubsystemBase {
    SwerveModuleIO blModule, brModule, tlModule, trModule;
    IMU imu;

    public DriveTrainSubsystem(SwerveModuleIO blModule, SwerveModuleIO brModule,
            SwerveModuleIO tlModule, SwerveModuleIO trModule, IMU imu) {
        this.blModule = blModule;
        this.brModule = brModule;
        this.tlModule = tlModule;
        this.trModule = trModule;
        this.imu = imu;
    }

    // TODO: Implement me!
}
