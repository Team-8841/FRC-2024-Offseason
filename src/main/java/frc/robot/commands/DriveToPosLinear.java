package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveTrainSubsystem;

/**
 * Drives to a position in a straight line.
 */
public class DriveToPosLinear extends CommandBase {
    private double endingSpeed, tolerance;
    private long startTime;
    private Translation2d endingTrans;

    private DriveTrainSubsystem driveTrain;

    private static double lerp(double t, double start, double end) {
        return (1 - t) * start + t * end;
    }

    public DriveToPosLinear(double endingSpeed, Translation2d endingTrans, double tolerance,
            DriveTrainSubsystem driveTrain) {
        this.endingSpeed = endingSpeed;
        this.endingTrans = endingTrans;
        this.tolerance = tolerance;

        this.driveTrain = driveTrain;

        this.addRequirements(driveTrain);
    }

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

        double t = (logger.getTimestamp() - this.startTime) / 1000000.0;
        double calculatedSpeed = lerp(t / Constants.accelerationTime, tolerance, endingSpeed);
        calculatedSpeed = Math.max(calculatedSpeed, this.endingSpeed);

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
