package frc.robot.sensors.imu;

/**
 * Container object for the pitch, yaw, and roll held by the IMU at a certain
 * time.
 */
public class Orientation {
    public double pitch, yaw, roll;

    public Orientation(double pitch, double yaw, double roll) {
        this.pitch = pitch;
        this.yaw = yaw;
        this.roll = roll;
    }
}
