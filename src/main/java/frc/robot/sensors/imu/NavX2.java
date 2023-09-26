package frc.robot.sensors.imu;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

/**
 * IMU Container for the NavX2 IMU.
 */
public class NavX2 implements IMU {
    private AHRS ahrs;

    public NavX2() {
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
