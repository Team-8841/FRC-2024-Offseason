package frc.robot.sensors.imu;

public class BNO055 implements IMUBase {
    public BNO055() {
    }

    public Orientation getOrientation() {
        // TODO: Implement me
        return new Orientation(0, 0, 0);
    }
}
