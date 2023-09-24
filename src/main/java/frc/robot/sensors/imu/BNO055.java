package frc.robot.sensors.imu;

/**
 * Intermediary implementation and glue code for the NavX2 IMU. Instead of using
 * this directly, please use {@link IMU} instead.
 */
public class BNO055 extends IMUBase {
    public BNO055() {
    }

    Orientation getOrientation() {
        // TODO: Implement me
        return new Orientation(0, 0, 0);
    }
}
