package frc.robot.sensors.imu;

/**
 * The main IMU singleton. This is a glue layer between an {@link IMUBase}
 * implementor and the main robot code. Please use this class whenever you want
 * to get data from the IMU.
 */
public final class IMU {
    private static IMU instance;

    private IMUBase internalSensor;

    private IMU(IMUBase internalSensor) {
        this.internalSensor = internalSensor;
    }

    /**
     * Initializes the singleton with a specific IMU. Must be called before calling
     * {@link getInstance}. If called twice, an error is thrown.
     * @param sensor IMUBase instance to wrap.
     */
    public static void initializeWithSensor(IMUBase sensor) {
        if (IMU.instance != null) {
            throw new Error("IMU has already been initialized!");
        }

        IMU.instance = new IMU(sensor);
    }

    /**
     * Singleton accessor.
     * @return The IMU singleton.
     */
    public static IMU getInstance() {
        if (IMU.instance == null) {
            throw new Error("IMU has not yet been initalized!");
        }

        return IMU.instance;
    }

    /**
     * @return Current orientation of the IMU.
     */
    public Orientation getOrientation() {
        return internalSensor.getOrientation();
    }

    /**
     * @return Current pitch of the IMU.
     */
    public double getPitch() {
        return internalSensor.getOrientation().pitch;
    }

    /**
     * @return Current yaw of the IMU.
     */
    public double getYaw() {
        return internalSensor.getOrientation().yaw;
    }

    /**
     * @return Current roll of the IMU.
     */
    public double getRoll() {
        return internalSensor.getOrientation().roll;
    }
}
