package frc.robot.sensors.imu;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

/**
 * Intermediary implementation and glue code for the NavX2 IMU. Instead of using
 * this directly, please use {@link IMU} instead.
 */
public class NavX2 implements IMU {
    private AHRS ahrs;

    private NavX2() {
        this.ahrs = new AHRS(SPI.Port.kMXP);
    }

    @Override
    public Orientation getOrientation() {
        return new Orientation(this.ahrs.getPitch(), this.ahrs.getFusedHeading(), this.ahrs.getRoll());
    }

    @Override
    public boolean isInitialized() {
        return true;
    }

    @Override
    public boolean isSensorPresent() {
        return this.ahrs.isConnected();
    }
}
