package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.imu.IMU;

/**
 * Subsystem which controls all sweve modules.
 */
public class SweveDriveTrainSubsystem extends SubsystemBase {
    private SwerveModuleContainer blModule, brModule, tlModule, trModule;
    private IMU imu;

    public SweveDriveTrainSubsystem(SwerveModuleContainer blModule, SwerveModuleContainer brModule,
            SwerveModuleContainer tlModule, SwerveModuleContainer trModule, IMU imu) {
        this.blModule = blModule;
        this.brModule = brModule;
        this.tlModule = tlModule;
        this.trModule = trModule;
        this.imu = imu;
    }

    // TODO: Implement me!
}
