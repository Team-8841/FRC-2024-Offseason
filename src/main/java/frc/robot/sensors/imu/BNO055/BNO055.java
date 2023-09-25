package frc.robot.sensors.imu.BNO055;

import frc.robot.sensors.imu.IMU;
import frc.robot.sensors.imu.IMUBase;
import frc.robot.sensors.imu.Orientation;

/**
 * Intermediary implementation and glue code for the BNO055 IMU. Instead of
 * using this directly, please use {@link IMU} instead.
 */
public class BNO055 implements IMUBase {
    private static BNO055 instance;

    private BNO055IO internalBNO055;

    private BNO055() {
        this.internalBNO055 = BNO055IO.getInstance(BNO055IO.opmode_t.OPERATION_MODE_IMUPLUS, BNO055IO.vector_type_t.VECTOR_EULER);
    }

    /**
     * Singleton accessor.
     * 
     * @return The NavX2 singleton to inject into {@link IMU}.
     */
    public static BNO055 getInstance() {
        if (BNO055.instance == null) {
            BNO055.instance = new BNO055();
        }

        return BNO055.instance;
    }

    public Orientation getOrientation() {
        double[] vector = this.internalBNO055.getVector();
        
        return new Orientation(vector[2], vector[1], vector[0]);
    }
}
