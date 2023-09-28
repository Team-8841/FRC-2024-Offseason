package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.imu.IMU;

/**
 * Main swerve drive class, interfaces with a hardware IO class.
 */
public class DriveTrainSubsystem extends SubsystemBase {
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModuleIO[] swerveModules;
    private IMU imu;

    public DriveTrainSubsystem(SwerveModuleIO[] swerveModules, IMU imu) {
        this.swerveModules = swerveModules;
        this.imu = imu;

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1);
        this.resetModules();

        this.swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, imu.getYaw(), this.getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    this.imu.getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].setDesiredState(swerveModuleStates[i]);
        }
    }    

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState states[] = new SwerveModuleState[4];

        for (int i = 0; i < this.swerveModules.length; i++) {
            states[i] = this.swerveModules[i].getState();
        }

        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition positions[] = new SwerveModulePosition[4];

        for (int i = 0; i < this.swerveModules.length; i++) {
            positions[i] = this.swerveModules[i].getPosition();
        }

        return positions;
    }

    public void resetModules() {
        for (SwerveModuleIO swerveModule : this.swerveModules) {
            swerveModule.reset();
        }
    }

    @Override
    public void periodic() {
        for (SwerveModuleIO swerveModule : this.swerveModules) {
            swerveModule.periodic();
        }
    }
}
