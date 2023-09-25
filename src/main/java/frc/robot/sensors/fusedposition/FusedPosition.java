package frc.robot.sensors.fusedposition;

/**
 * Fuses the data from the encoders, the IMU, and vision using a kalman filter
 * in order to get a best a guess of the robot's current position and
 * orientation.
 */
public class FusedPosition {
    private static FusedPosition instance;

    // TODO: Implement me.

    private FusedPosition() {}

    /**
     * Singleton accessor.
     * 
     * @return The FusedPosition singleton.
     */
    public FusedPosition getInstance() {
        if (FusedPosition.instance == null) {
            FusedPosition.instance = new FusedPosition();
        }

        return FusedPosition.instance;
    }
}
