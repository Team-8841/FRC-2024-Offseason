package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveTrainSubsystem;

/**
 * Drives to a position in a straight line at a certain velocity and containues
 * at that velocity once it hits the desired waypoint. This is useful for
 * driving to the beginning of paths.
 */
public class DriveToPosLinear extends CommandBase {
    private double startingSpeed, endingSpeed, tolerance;
    private long startTime;
    private Translation2d endingTrans;

    private DriveTrainSubsystem driveTrain;

    /**
     * @param endingSpeed The speed we want to drive to the ending translation with and to continue at.
     * @param endingTrans The field relative translation to drive too.
     * @param tolerance   How close do we want to be from the ending translation?
     * @param driveTrain  The drivetrain subsystem.
     */
    public DriveToPosLinear(double endingSpeed, Translation2d endingTrans, double tolerance,
            DriveTrainSubsystem driveTrain) {
        this.endingSpeed = endingSpeed;
        this.endingTrans = endingTrans;
        this.tolerance = tolerance;

        this.driveTrain = driveTrain;

        this.startingSpeed = driveTrain.getLinearSpeed();

        this.addRequirements(driveTrain);
    }

    /**
     * Gets the translation representing the difference in position from the robot
     * and the desired position.
     * 
     * @return Translation2d containing the position deltas.
     */
    private Translation2d getDeltaTrans() {
        Translation2d currentTrans = this.driveTrain.getPose().getTranslation();
        return new Translation2d(
                this.endingTrans.getX() - currentTrans.getX(),
                this.endingTrans.getY() - currentTrans.getY());
    }

    @Override
    public void execute() {
        Logger logger = Logger.getInstance();

        if (this.startTime == 0) {
            this.startTime = logger.getTimestamp();
        }

        // Interpolate the speed so that we only accelerate at Constants.maxAcceleration.
        double t = (logger.getTimestamp() - this.startTime) / 1000000.0;
        double calculatedSpeed = Math.max(this.startingSpeed + t * Constants.maxAcceleration, this.endingSpeed);

        Translation2d deltaTrans = this.getDeltaTrans();
        Translation2d directionTrans = deltaTrans.div(deltaTrans.getNorm());

        this.driveTrain.drive(directionTrans.times(calculatedSpeed), 0, true);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // Probably a good idea to shut off the motors
            this.driveTrain.drive(new Translation2d(0, 0), 0, false);
        }
    }

    @Override
    public boolean isFinished() {
        return this.getDeltaTrans().getNorm() < this.tolerance;
    }
}
