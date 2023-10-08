package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveTrainSubsystem;

/**
 * Drives to a position in a straight line at a certain velocity and containues
 * at that velocity once it hits the desired waypoint. This is useful for
 * driving to the beginning of paths.
 */
public class DriveToPosLinear extends CommandBase {
    private double tolerance;
    private Translation2d endingTrans;

    private TrapezoidProfile.State goalState;
    private TrapezoidProfile.Constraints constraints;

    private DriveTrainSubsystem driveTrain;

    /**
     * @param endingSpeed The speed we want to drive to the ending translation with
     *                    and to continue at.
     * @param endingTrans The field relative translation to drive too.
     * @param tolerance   How close do we want to be from the ending translation?
     * @param driveTrain  The drivetrain subsystem.
     */
    public DriveToPosLinear(double endingSpeed, Translation2d endingTrans, double tolerance,
            DriveTrainSubsystem driveTrain) {
        this.addRequirements(driveTrain);

        this.endingTrans = endingTrans;
        this.tolerance = tolerance;

        this.driveTrain = driveTrain;

        this.goalState = new TrapezoidProfile.State(0, endingSpeed);
        this.constraints = new TrapezoidProfile.Constraints(Constants.AutoConstants.MaxSpeedMetersPerSecond,
                Constants.AutoConstants.MaxAccelerationMetersPerSecondSquared);
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

    public void updateProfile() {
    }

    @Override
    public void execute() {
        Translation2d deltaTrans = this.getDeltaTrans();
        Translation2d dirTrans = deltaTrans.div(deltaTrans.getNorm());

        TrapezoidProfile.State currentState = new TrapezoidProfile.State(-deltaTrans.getNorm(),
                this.driveTrain.getLinearSpeed());
        TrapezoidProfile profile = new TrapezoidProfile(this.constraints, this.goalState, currentState);
        TrapezoidProfile.State applyState = profile.calculate(Constants.Swerve.approxDriveInputDelay);

        this.driveTrain.drive(dirTrans.times(applyState.velocity), 0, true);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // Probably a good idea to shut off the motors
            this.driveTrain.drive(new Translation2d(0, 0), 0, false);
        } else {
            Translation2d deltaTrans = this.getDeltaTrans();
            this.driveTrain.drive(deltaTrans.times(this.goalState.velocity / deltaTrans.getNorm()), 0, true);
        }
    }

    @Override
    public boolean isFinished() {
        return this.getDeltaTrans().getNorm() <= this.tolerance;
    }
}
