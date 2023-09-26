package frc.robot.sensors.imu.BNO055;

import frc.robot.sensors.imu.IMU;
import frc.robot.sensors.imu.Orientation;

/**
 * IMU container for the BNO055.
 */
public class BNO055 implements IMU {
    private BNO055IO internalBNO055;

    public BNO055() {
        this.internalBNO055 = BNO055IO.getInstance(BNO055IO.opmode_t.OPERATION_MODE_IMUPLUS,
                BNO055IO.vector_type_t.VECTOR_EULER);
    }

    @Override
    public Orientation getOrientation() {
        double[] vector = this.internalBNO055.getVector();

        return new Orientation(vector[2], vector[1], vector[0]);
    }

    @Override
    public boolean isInitialized() {
        return this.internalBNO055.isInitialized();
    }

    @Override
    public boolean isSensorPresent() {
        return this.internalBNO055.isSensorPresent();
    }
}
