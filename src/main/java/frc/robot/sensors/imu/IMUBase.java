package frc.robot.sensors.imu;

public interface IMUBase {
    public default void initSensor() {
    }

    public Orientation getOrientation();
}
