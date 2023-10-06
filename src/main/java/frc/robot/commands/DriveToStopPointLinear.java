package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveTrainSubsystem;

/**
 * Drives to a position in a straight line and then stops. This is accomplished
 * by using two seperate PIDs for the x and y position of the robot.
 */
public class DriveToStopPointLinear extends CommandBase {
    private double tolerance;
    private boolean feedforward;
    private Translation2d endingTrans;

    private DriveTrainSubsystem driveTrain;

    private PIDController xPID = new PIDController(Constants.positionKP, Constants.positionKI, Constants.positionKD);
    private PIDController yPID = new PIDController(Constants.positionKP, Constants.positionKI, Constants.positionKD);
    private SlewRateLimiter speedLimiter;

    /**
     * @param endingTrans The field relative translation to drive too.
     * @param feedforward Whether or not we want to use a feedforward term on the
     *                    velocity (see DriveToStopPointLinear.java:67 for details).
     * @param tolerance   How close do we want to be from the ending translation?
     * @param driveTrain  The drivetrain subsystem.
     */
    public DriveToStopPointLinear(Translation2d endingTrans, boolean feedforward, double tolerance,
            DriveTrainSubsystem driveTrain) {
        this.endingTrans = endingTrans;
        this.feedforward = feedforward;
        this.tolerance = tolerance;

        this.driveTrain = driveTrain;

        this.addRequirements(driveTrain);

        this.speedLimiter = new SlewRateLimiter(Constants.maxAcceleration, -Constants.maxAcceleration,
                driveTrain.getLinearSpeed());
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
        Translation2d deltaTrans = this.getDeltaTrans();

        // The PID's return negative values from what we want, so we multiply them by
        // -1.
        double xSpeed = -this.xPID.calculate(deltaTrans.getX());
        double ySpeed = -this.yPID.calculate(deltaTrans.getY());

        Translation2d velocity = new Translation2d(xSpeed, ySpeed);

        // Because of the nature of PID's, if we have a small tolerance set, it'll take
        // a while to actually reach the desired position since the error keeps getting
        // smaller. So, a feedforward fixes this by adding a constant velocity to the
        // PID's output, so that we don't slow down to a crawl at the end and actually
        // reach our desired position in a reasonable amount of time. Once we get
        // withing tolerance of the desired position, isFinished() returns true and the
        // motors are shut off so we should get within tolerance (atleast for
        // large(-ish) values).
        if (this.feedforward) {
            Translation2d feedforwardVelocity = deltaTrans.times(Constants.positionKS / deltaTrans.getNorm());
            velocity = velocity.plus(feedforwardVelocity);
        }

        // Desaturates the desired velocity with the value from the slew rate limiter.
        velocity = velocity.times(this.speedLimiter.calculate(velocity.getNorm()) / velocity.getNorm());

        this.driveTrain.drive(velocity, 0, true);
    }

    @Override
    public void end(boolean interrupted) {
        this.driveTrain.drive(new Translation2d(0, 0), 0, false);
    }

    @Override
    public boolean isFinished() {
        return this.getDeltaTrans().getNorm() < this.tolerance;
    }
}
