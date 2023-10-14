package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.CTREConfigs;
import frc.robot.Constants;

public class TalonFXSwerveModuleIO implements SwerveModuleIO {
    private TalonFX driveMotor, steeringMotor;

    private Rotation2d lastAngle;

    // Added to the drive motor's closed loop PID output to maintain velocity so
    // that the PID doesn't constantly have to readjust for error
    SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(Constants.Swerve.PureTalonFX.driveKS,
            Constants.Swerve.PureTalonFX.driveKV, Constants.Swerve.PureTalonFX.driveKA);

    SwerveModuleConstants constants;

    /**
     * Creates a new container from each of the components.
     * 
     * @param driveMotor      The TalonFX controller controlling the drive motor.
     * @param steeringMotor   The SparkMax controller controlling the steering
     *                        motor.
     * @param steeringEncoder The CTRE CANCoder attached to the steering motor.
     */
    public TalonFXSwerveModuleIO(TalonFX driveMotor, TalonFX steeringMotor, SwerveModuleConstants constants) {
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.constants = constants;

        this.configDriveMotor();
        this.configSteeringMotor();
        this.resetAbsolutePosition();
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
    public TalonFXSwerveModuleIO(SwerveModuleConstants constants) {
        this(
                new TalonFX(constants.driveMotorID),
                new TalonFX(constants.angleMotorID),
                constants);
    }

    private void configDriveMotor() {
        this.driveMotor.configFactoryDefault();
        this.driveMotor.configAllSettings(CTREConfigs.swerveDriveFXConfig);
        this.driveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        this.driveMotor.setNeutralMode(Constants.Swerve.PureTalonFX.driveNeutralMode);
        this.driveMotor.setSelectedSensorPosition(0);
    }

    private void configSteeringMotor() {
        this.steeringMotor.configFactoryDefault();
        this.steeringMotor.configAllSettings(CTREConfigs.swerveAngleFXConfig);
        this.steeringMotor.setInverted(Constants.Swerve.driveMotorInvert);
        this.steeringMotor.setNeutralMode(Constants.Swerve.PureTalonFX.angleNeutralMode);
        this.steeringMotor.configRemoteFeedbackFilter(this.constants.cancoderID, RemoteSensorSource.CANCoder, 0);
        this.steeringMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 1000);
        this.steeringMotor.setSelectedSensorPosition(0);
    }

    private void resetAbsolutePosition() {
        double angle = this.getAngle().getDegrees() - this.constants.angleOffset.getDegrees();
        this.steeringMotor.setSelectedSensorPosition(angle);
    }

    @Override
    public void reset() {
        this.resetAbsolutePosition();
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState) {
        // Closed loop
        double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
                Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        this.driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                driveFeedforward.calculate(desiredState.speedMetersPerSecond));
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;

        this.lastAngle = angle;

        this.steeringMotor.set(TalonFXControlMode.Velocity, Conversions.degreesToCANcoder(angle.getDegrees(), 1));
    }

    @Override
    public SwerveModuleState getState() {
        double angle = Conversions.CANcoderToDegrees(this.steeringMotor.getSelectedSensorPosition(), 1);

        return new SwerveModuleState(
                Conversions.falconToMPS(this.driveMotor.getSelectedSensorVelocity(),
                        Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio),
                Rotation2d.fromDegrees(angle));
    }

    @Override
    public SwerveModulePosition getPosition() {
        double angle = Conversions.CANcoderToDegrees(this.steeringMotor.getSelectedSensorPosition(), 1);

        return new SwerveModulePosition(
                Conversions.falconToMeters(this.driveMotor.getSelectedSensorPosition(),
                        Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio),
                Rotation2d.fromDegrees(angle));
    }

    @Override
    public void initializeShuffleBoardLayout(ShuffleboardLayout layout) {
        layout.addDouble("Angle", () -> this.getAngle().getDegrees());
        layout.addDouble("Speed", this::getSpeed);
        layout.addDouble("Position", () -> this.getPosition().distanceMeters);
        layout.addDouble("Drive Motor Stator Current", this.driveMotor::getStatorCurrent);
        layout.addDouble("Drive Motor Supply Current", this.driveMotor::getSupplyCurrent);
        layout.addDouble("Steering Motor Stator Current", this.steeringMotor::getStatorCurrent);
        layout.addDouble("Steering Motor Supply Current", this.steeringMotor::getSupplyCurrent);
    }
}
