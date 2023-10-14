package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.CubicBezier;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveTrainSubsystem;

public class DriveSubPath extends CommandBase {
    protected DriveTrainSubsystem driveTrain;

    private TrapezoidProfile.Constraints motionMaxConstraints;
    private TrapezoidProfile.State motionGoal;

    private CubicBezier path;
    private CubicBezier.LUT pathLUT;

    private double tolerance, endingVelocity;

    public DriveSubPath(CubicBezier path, double tolerance, double endingVelocity, DriveTrainSubsystem driveTrain) {
        this.addRequirements(driveTrain);

        this.driveTrain = driveTrain;

        this.tolerance = tolerance;
        this.endingVelocity = endingVelocity;

        this.path = path;
        this.pathLUT = path.getLUT();

        this.motionMaxConstraints = new TrapezoidProfile.Constraints(Constants.AutoConstants.MaxSpeedMetersPerSecond,
                Constants.AutoConstants.MaxAccelerationMetersPerSecondSquared);
        this.motionGoal = new TrapezoidProfile.State(this.pathLUT.getDistAtT(1), this.endingVelocity);
    }

    @Override
    public void execute() {
        Pose2d pose = this.driveTrain.getPose();

        // TODO: Eventually change this into a proper implementation of pure pursuit
        double closestT = this.pathLUT.getClosestT(pose.getTranslation());
        double currentDist = this.pathLUT.getDistAtT(closestT);
        double lookaheadT = this.pathLUT
                .getTAtDist(currentDist + Constants.AutoConstants.pathingLookaheadDistance);

        Translation2d lookahead = this.path.get(lookaheadT);
        Translation2d lookaheadDelta = lookahead.minus(pose.getTranslation());

        TrapezoidProfile.Constraints adjustedConstraints = new TrapezoidProfile.Constraints(
            // Slows down the robot around areas of high curvature (like tight corners)
            Math.max(this.motionMaxConstraints.maxVelocity / (1 + this.path.getCurvature(closestT)), 0.1),
            this.motionMaxConstraints.maxAcceleration
        );

        TrapezoidProfile.State currentState = new TrapezoidProfile.State(
            this.motionGoal.position - currentDist,
            this.driveTrain.getLinearSpeed()
        );

        Logger.getInstance().recordOutput("/Pathing/CurrentPos", currentDist);
        Logger.getInstance().recordOutput("/PathingGoalPos", this.motionGoal.position);
        Logger.getInstance().recordOutput("/Pathing/EndingVel", this.motionGoal.velocity);

        TrapezoidProfile motionProfile = new TrapezoidProfile(adjustedConstraints, this.motionGoal, currentState);

        double calculatedSpeed = Math.abs(motionProfile.calculate(Constants.Swerve.approxDriveInputDelay).velocity);
        Translation2d velocity = lookaheadDelta.times(calculatedSpeed / lookaheadDelta.getNorm());

        // Makes robot go zooom :3
        this.driveTrain.drive(velocity, 0, true);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // Probably a good idea to shut off the motors
            this.driveTrain.drive(new Translation2d(), 0, false);
        }
    }

    @Override
    public boolean isFinished() {
        Translation2d currentPos = this.driveTrain.getPose().getTranslation();
        double endingDistDelta = this.pathLUT.getDistAtT(1) - this.pathLUT.getDistAtT(this.pathLUT.getClosestT(currentPos));

        return Math.abs(endingDistDelta) <= this.tolerance;
    }
}
