package frc.robot.sensors.imu;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.SPI;

/**
 * IMU Container for the NavX2 IMU.
 */
public class NavX2 extends IMU {
    private AHRS ahrs;

    public NavX2() {
        this.ahrs = new AHRS(SPI.Port.kMXP);
        this.ahrs.zeroYaw();
    }

    @Override
    public Rotation3d getOrientation() {
        return new Rotation3d(new Quaternion(this.ahrs.getQuaternionW(), this.ahrs.getQuaternionX(),
                this.ahrs.getQuaternionY(), this.ahrs.getQuaternionZ()));
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
