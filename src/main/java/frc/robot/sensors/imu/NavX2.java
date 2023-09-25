package frc.robot.sensors.imu;

import com.kauailabs.navx.frc.AHRS;

/**
 * Intermediary implementation and glue code for the NavX2 IMU. Instead of using
 * this directly, please use {@link IMU} instead.
 */
public class NavX2 extends IMUBase {
    private static NavX2 instance;

    private AHRS ahrs;

    private NavX2() {
        // Idk how we actually connected the navx to the rio, soooo
        // TODO: Figure out how it's actually connected and correct this.
        this.ahrs = new AHRS();
    }

    public NavX2 getInstance() {
        if (this.instance == null) {
            this.instance = new NavX2();
        }

        return this.instance;
    }

    Orientation getOrientation() {
        // TODO: Implement me
        return new Orientation(this.ahrs.getPitch(), this.ahrs.getFusedHeading(), this.ahrs.getRoll());
    }
}
