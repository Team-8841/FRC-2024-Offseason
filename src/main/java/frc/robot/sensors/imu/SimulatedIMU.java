package frc.robot.sensors.imu;

import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Simulated IMU used during simulations.
 */
public class SimulatedIMU extends IMU {
    @Override
    public Rotation3d getOrientation() {
        return new Rotation3d();
    }

    @Override
    public boolean isSensorPresent() {
        return true;
    }

    @Override
    public boolean isInitialized() {
        return true;
    }
}
