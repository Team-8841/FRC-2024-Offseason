// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopSwerve;
import frc.robot.sensors.imu.IMU;
import frc.robot.sensors.imu.NavX2;
import frc.robot.sensors.imu.SimIMU;
import frc.robot.subsystems.drive.DriveTrainSubsystem;
import frc.robot.subsystems.drive.TalonFXSwerveModuleIO;
import frc.robot.subsystems.drive.SimSwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIO;

public class RobotContainer {
  // Sensors
  private IMU imu;

  // Subsystems
  private DriveTrainSubsystem sweveDriveTrain;

  // Controllers
  private CommandXboxController driveController;

  public RobotContainer() {
    SwerveModuleIO swerveModules[];

    if (RobotBase.isReal()) {
      swerveModules = new SwerveModuleIO[] {
          new TalonFXSwerveModuleIO(Constants.Swerve.PureTalonFX.Mod0.constants),
          new TalonFXSwerveModuleIO(Constants.Swerve.PureTalonFX.Mod1.constants),
          new TalonFXSwerveModuleIO(Constants.Swerve.PureTalonFX.Mod2.constants),
          new TalonFXSwerveModuleIO(Constants.Swerve.PureTalonFX.Mod3.constants),
      };
      this.imu = new NavX2();
    } else {
      swerveModules = new SwerveModuleIO[] {
          new SimSwerveModuleIO(),
          new SimSwerveModuleIO(),
          new SimSwerveModuleIO(),
          new SimSwerveModuleIO(),
      };

      this.imu = new SimIMU();
    }

    this.sweveDriveTrain = new DriveTrainSubsystem(swerveModules, this.imu);

    ShuffleboardTab robotTab = Shuffleboard.getTab("Robot");
    this.imu.initializeShuffleBoardLayout(robotTab.getLayout("IMU", BuiltInLayouts.kList));

    this.driveController = new CommandXboxController(0);
    this.configureBindings(this.driveController);
  }

  private void configureBindings(CommandXboxController controller) {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Command getTeleopCommand() {
    return new TeleopSwerve(sweveDriveTrain, () -> -this.driveController.getLeftY(),
        () -> -this.driveController.getLeftX(),
        () -> -this.driveController.getRightX());
  }
}
