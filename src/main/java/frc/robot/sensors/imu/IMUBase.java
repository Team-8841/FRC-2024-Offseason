package frc.robot.sensors.imu;

public abstract class IMUBase {
    void initSensor() {
    }

    abstract Orientation getOrientation();
}