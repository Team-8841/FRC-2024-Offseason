package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;

/**
 * Container 
 */
public class SwerveModuleContainer {
    public TalonFX driveMotor;
    public CANSparkMax steeringMotor;
    public CANCoder steeringEncoder;

    public SwerveModuleContainer(TalonFX driveMotor, CANSparkMax steeringMotor, CANCoder steeringEncoder) {
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.steeringEncoder = steeringEncoder;
    }
}
