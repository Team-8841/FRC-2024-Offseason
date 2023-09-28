package frc.robot.sensors.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Interface used by all IMU's. All implementing classes HAVE TO output
 * orientations as CCW+ (Counter-clockwise positive).
 */
public interface IMU {
    /**
     * Get's the IMU's current orientation.
     * 
     * @return The current 3d orientation (pitch, roll, and yaw) of the IMU as a
     *         Rotation3d.
     */
    public Rotation3d getOrientation();

    /**
     * Gets the IMU's current pitch angle.
     * 
     * @return The current pitch angle as a Rotation2d
     */
    public default Rotation2d getPitch() {
        return Rotation2d.fromRadians(this.getOrientation().getY());
    }

    /**
     * Gets the IMU's current yaw angle.
     * 
     * @return The current yaw angle as a Rotation2d.
     */
    public default Rotation2d getYaw() {
        return Rotation2d.fromRadians(this.getOrientation().getZ());
    }

    /**
     * Gets the IMU's current roll angle.
     * 
     * @return The current roll angle as a Rotation2d.
     */
    public default Rotation2d getRoll() {
        return Rotation2d.fromRadians(this.getOrientation().getX());
    }

    /**
     * Get's if the IMU has been initialized.
     * 
     * @return If the IMU has been initialized.
     */
    public boolean isInitialized();

    /**
     * Get's if the IMU is present.
     * 
     * @return If the IMU is present.
     */
    public boolean isSensorPresent();
}