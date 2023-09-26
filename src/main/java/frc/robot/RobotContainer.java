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
import frc.robot.subsystems.SwerveModuleContainer;
import frc.robot.subsystems.SweveDriveTrainSubsystem;

public class RobotContainer {
  // Sensors
  private IMU imu;

  // Subsystems
  private SweveDriveTrainSubsystem sweveDriveTrainSubsystem;

  public RobotContainer() {
    SwerveModuleContainer blModule, brModule, tlModule, trModule;

    blModule = new SwerveModuleContainer(Constants.SWERVE_BL_TALON_DRIVE_CAN_ID,
        Constants.SWERVE_BL_SPARK_STEERING_CAN_ID, Constants.SWERVE_BL_CANCODER_CAN_ID);
    brModule = new SwerveModuleContainer(Constants.SWERVE_BR_TALON_DRIVE_CAN_ID,
        Constants.SWERVE_BR_SPARK_STEERING_CAN_ID, Constants.SWERVE_BR_CANCODER_CAN_ID);
    tlModule = new SwerveModuleContainer(Constants.SWERVE_TL_TALON_DRIVE_CAN_ID,
        Constants.SWERVE_TL_SPARK_STEERING_CAN_ID, Constants.SWERVE_TL_CANCODER_CAN_ID);
    trModule = new SwerveModuleContainer(Constants.SWERVE_TR_TALON_DRIVE_CAN_ID,
        Constants.SWERVE_TR_SPARK_STEERING_CAN_ID, Constants.SWERVE_TR_CANCODER_CAN_ID);

    if (RobotBase.isReal()) {
      this.imu = new NavX2();
    }
    else {
      this.imu = new SimulatedIMU();
    }

    this.sweveDriveTrainSubsystem = new SweveDriveTrainSubsystem(blModule, brModule, tlModule, trModule, this.imu);

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
