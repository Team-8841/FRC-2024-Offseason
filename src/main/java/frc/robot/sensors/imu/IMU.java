package frc.robot.sensors.imu;

/**
 * Interface used by all IMU's.
 */
public interface IMU {
    /**
     * Get's the IMU's current orientation.
     * 
     * @return The current 3d orientation (pitch, roll, and yaw) of the IMU. All
     *         0-360 degrees
     */
    public Orientation getOrientation();

    /**
     * Gets the IMU's current pitch angle.
     * 
     * @return The current pitch angle. 0-360 degrees
     */
    public default double getPitch() {
        return this.getOrientation().pitch;
    }

    /**
     * Gets the IMU's current yaw angle.
     * 
     * @return The current yaw angle. 0-360 degrees
     */
    public default double getYaw() {
        return this.getOrientation().yaw;
    }

    /**
     * Gets the IMU's current roll angle.
     * 
     * @return The current roll angle. 0-360 degrees
     */
    public default double getRoll() {
        return this.getOrientation().roll;
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