package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public Rotation2d steeringMotorAngle;
        public double steeringMotorVelocity;
        public double steeringMotorPosition;

        public double speed;
        public double driveMotorVelocity;
        public double driveMotorPosition;
    }

    /**
     * Called on every subsystem periodic.
     */
    public default void periodic() {
    }

    /**
     * Resets the module to a known state.
     */
    public default void reset() {
    }

    public default void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
    }

    /**
     * Set the desired state of the swerve module.
     * 
     * @param desiredState The desired state.
     */
    public default void setDesiredState(SwerveModuleState desiredState) {
        this.setAngle(desiredState);
        this.setSpeed(desiredState);
    }

    /**
     * Set the angle of the swerve module.
     * 
     * @param desiredState State which contains the desired angle.
     */
    public void setAngle(SwerveModuleState desiredState);

    /**
     * Set the speed of the swerve module.
     * 
     * @param desiredState State which contains the desired speed.
     */
    public void setSpeed(SwerveModuleState desiredState);

    /**
     * Gets the current state of the swerve module.
     * 
     * @return The current state.
     */
    public SwerveModuleState getState();

    /**
     * Gets the current position of the swerve module.
     * 
     * @return The current position.
     */
    public SwerveModulePosition getPosition();

    /**
     * Gets the current state of the swerve module.
     * 
     * @return The current state.
     */
    public default Rotation2d getAngle() {
        return this.getState().angle;
    }

    /**
     * Gets the current state of the swerve module.
     * 
     * @return The current state.
     */
    public default double getSpeed() {
        return this.getState().speedMetersPerSecond;
    }
}