package frc.robot.sensors.imu;

/**
 * Simulated IMU used during simulations.
 */
public class SimulatedIMU implements IMU {
    @Override
    public Orientation getOrientation() {
        return new Orientation(0, 0, 0);
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
