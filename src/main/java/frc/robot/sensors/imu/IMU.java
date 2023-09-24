package frc.robot.sensors.imu;

public final class IMU {
    private static IMU instance;

    private IMUBase internalSensor;

    private IMU(IMUBase internalSensor) {
        this.internalSensor = internalSensor;
    }

    public static void initializeWithSensor(IMUBase sensor) {
        if (IMU.instance != null) {
            throw new Error("IMU has already been initialized!");
        }

        IMU.instance = new IMU(sensor);
    }

    public static IMU getInstance() {
        if (IMU.instance == null) {
            throw new Error("IMU has not yet been initalized!");
        }

        return IMU.instance;
    }

    public Orientation getOrientation() {
        return internalSensor.getOrientation();
    }

    public double getPitch() {
        return internalSensor.getOrientation().pitch;
    }

    public double getYaw() {
        return internalSensor.getOrientation().yaw;
    }

    public double getRoll() {
        return internalSensor.getOrientation().roll;
    }
}
