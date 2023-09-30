package frc.robot.sensors.imu;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.SimManager;

/**
 * Simulated IMU used during simulations.
 */
public class SimulatedIMU extends IMU {
    private Supplier<Rotation2d> yawSupplier;

    public SimulatedIMU() {
        SimManager.getInstance().registerIMU(this);
    }

    public void registerOrientationSuppliers(Supplier<Rotation2d> yawSupplier) {
        this.yawSupplier = yawSupplier;
    }

    @Override
    public Rotation3d getOrientation() {
        if (this.yawSupplier == null) {
            return new Rotation3d();
        }

        return new Rotation3d(0, 0, this.yawSupplier.get().getRadians());
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
