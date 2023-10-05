package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveTrainSubsystem;

/**
 * Drives to a position in a straight line.
 */
public class DriveToStopPointLinear extends CommandBase {
    private double tolerance;
    private boolean feedforward;
    private Translation2d endingTrans;

    private DriveTrainSubsystem driveTrain;

    private PIDController xPID = new PIDController(Constants.positionKP, Constants.positionKI, Constants.positionKD);
    private PIDController yPID = new PIDController(Constants.positionKP, Constants.positionKI, Constants.positionKD);

    public DriveToStopPointLinear(Translation2d endingTrans, boolean feedforward, double tolerance,
            DriveTrainSubsystem driveTrain) {
        this.endingTrans = endingTrans;
        this.feedforward = feedforward;
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
        Translation2d deltaTrans = this.getDeltaTrans();

        double xSpeed = -this.xPID.calculate(deltaTrans.getX());
        double ySpeed = -this.yPID.calculate(deltaTrans.getY());

        Translation2d velocity = new Translation2d(xSpeed, ySpeed);
        
        if (this.feedforward) {
            Translation2d feedforwardVelocity = deltaTrans.times(Constants.positionKS / deltaTrans.getNorm());

            velocity = new Translation2d(
                velocity.getX() + feedforwardVelocity.getX(),
                velocity.getY() + feedforwardVelocity.getY()
            );
        }

        Logger.getInstance().recordOutput("/DesiredSpeed", velocity.getNorm());

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
