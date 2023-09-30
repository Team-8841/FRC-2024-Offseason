package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveTrainSubsystem;

public class TeleopSwerve extends CommandBase {
    private DoubleSupplier forwardSupplier, strafeSupplier, rotationSupplier;
    private DriveTrainSubsystem driveTrain;

    public TeleopSwerve(DriveTrainSubsystem driveTrain, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier,
            DoubleSupplier rotationSupplier) {
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        //this.strafeSupplier = () -> 0;
        //this.rotationSupplier = strafeSupplier;
        this.driveTrain = driveTrain;

        this.addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        Translation2d driveTranslation = new Translation2d(
                MathUtil.applyDeadband(forwardSupplier.getAsDouble(), Constants.controllerDeadband),
                MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.controllerDeadband));
        double rotation = MathUtil.applyDeadband(this.rotationSupplier.getAsDouble(), Constants.controllerDeadband);


        this.driveTrain.drive(driveTranslation, rotation, false);
    }
}
