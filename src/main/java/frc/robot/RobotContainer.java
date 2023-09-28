// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.sensors.imu.IMU;
import frc.robot.sensors.imu.NavX2;
import frc.robot.sensors.imu.SimulatedIMU;
import frc.robot.subsystems.drive.DriveTrainSubsystem;
import frc.robot.subsystems.drive.MixedSwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIO;

public class RobotContainer {
  // Sensors
  private IMU imu;

  // Subsystems
  private DriveTrainSubsystem sweveDriveTrainSubsystem;

  public RobotContainer() {
    SwerveModuleIO swerveModules[];

    if (RobotBase.isReal()) {
      swerveModules = new SwerveModuleIO[] {
        new MixedSwerveModuleIO(Constants.Swerve.Mod0.constants),
        new MixedSwerveModuleIO(Constants.Swerve.Mod1.constants),
        new MixedSwerveModuleIO(Constants.Swerve.Mod2.constants),
        new MixedSwerveModuleIO(Constants.Swerve.Mod3.constants),
      };

      this.imu = new NavX2();
    }
    else {
      /* 
      swerveModules = new SwerveModuleIO[] {
        new SimSwerveModuleIO(),
        new SimSwerveModuleIO(),
        new SimSwerveModuleIO(),
        new SimSwerveModuleIO(),
      };
      */

      swerveModules = new SwerveModuleIO[] {};

      this.imu = new SimulatedIMU();
    }

    this.sweveDriveTrainSubsystem = new DriveTrainSubsystem(swerveModules, this.imu);

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
