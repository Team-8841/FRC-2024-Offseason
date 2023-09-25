package frc.robot.sensors.imu;

/**
 * Base class for all IMU intermediaries. These are passed into {@link IMU} in
 * order to initialize the singleton which can then be used in the main robot
 * code.
 */
public interface IMUBase {
    public Orientation getOrientation();
}