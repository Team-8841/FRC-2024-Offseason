package frc.robot.sensors.driveencoders;

public final class DriveEncoders {
    // TODO: implement me

    private static DriveEncoders instance;

    private DriveEncoders() {
    }

    public static DriveEncoders getInstance() {
        if (DriveEncoders.instance == null) {
            DriveEncoders.instance = new DriveEncoders();
        }

        return DriveEncoders.instance;
    }
}
