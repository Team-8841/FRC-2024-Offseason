package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SimSwerveModuleIO implements SwerveModuleIO {
    public SimSwerveModuleIO() {
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
        // TODO: Implement me!
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState) {
        // TODO: Implement me!
    }

    @Override
    public SwerveModuleState getState() {
        // TODO: Implement me!
        return new SwerveModuleState();
    }

    @Override
    public SwerveModulePosition getPosition() {
        // TODO: Implement me!
        return new SwerveModulePosition();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        // TODO: Implement me!
    }
}