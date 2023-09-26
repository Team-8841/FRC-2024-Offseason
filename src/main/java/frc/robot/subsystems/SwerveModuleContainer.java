package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Container class which includes all components of a swerve module.
 */
public class SwerveModuleContainer {
    public TalonFX driveMotor;
    public CANSparkMax steeringMotor;
    public CANCoder steeringEncoder;

    /**
     * Creates a new container from each of the components.
     * 
     * @param driveMotor      The TalonFX controller controlling the drive motor.
     * @param steeringMotor   The SparkMax controller controlling the steering
     *                        motor.
     * @param steeringEncoder The CTRE CANCoder attached to the steering motor.
     */
    public SwerveModuleContainer(TalonFX driveMotor, CANSparkMax steeringMotor, CANCoder steeringEncoder) {
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.steeringEncoder = steeringEncoder;
    }

    /**
     * Creates a new container from each of the component's CAN ID's.
     * 
     * @param driveMotorCANID      The CAN ID of the TalonFX controlled drive motor.
     * @param steeringMotorCANID   The CAN ID of the SparkMax controlled steering
     *                             motor.
     * @param steeringEncoderCANID The CAN ID of the CTRE CANConder steering
     *                             encoder.
     */
    public SwerveModuleContainer(int driveMotorCANID, int steeringMotorCANID, int steeringEncoderCANID) {
        this.driveMotor = new TalonFX(driveMotorCANID);
        this.steeringMotor = new CANSparkMax(steeringMotorCANID, MotorType.kBrushless);
        this.steeringEncoder = new CANCoder(steeringEncoderCANID);
    }
}
