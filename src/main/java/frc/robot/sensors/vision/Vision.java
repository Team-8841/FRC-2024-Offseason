package frc.robot.sensors.vision;

/**
 * Singleton for interfacing with any limelights/photonvisions on the robot.
 */
public class Vision {
    private static Vision instance;

    // TODO: Implement me.

    private Vision() {}

    /**
     * Singleton accessor.
     * 
     * @return The Vision singleton.
     */
    public static Vision getInstance() {
        if (Vision.instance == null) {
            Vision.instance = new Vision();
        }

        return Vision.instance;
    }
}
