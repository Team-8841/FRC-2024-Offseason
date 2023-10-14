package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.CTREConfigs;
import frc.robot.Constants;

public class MixedSwerveModuleIO implements SwerveModuleIO {
    private TalonFX driveMotor;
    private CANSparkMax steeringMotor;
    private CANCoder steeringEncoder;

    private Rotation2d lastAngle;

    // I would've liked to use closed loop control on the neo but the cancoder can't
    // be attached to the sparkmax :/
    PIDController steeringPID = new PIDController(Constants.Swerve.Mixed.angleKP, Constants.Swerve.Mixed.angleKI,
                Constants.Swerve.Mixed.angleKD);

    // Added to the drive motor's closed loop PID output to maintain velocity so
    // that the PID doesn't constantly have to readjust for error
    SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(Constants.Swerve.Mixed.driveKS,
            Constants.Swerve.Mixed.driveKV, Constants.Swerve.Mixed.driveKA);

    SwerveModuleConstants constants;

    /**
     * Creates a new container from each of the components.
     * 
     * @param driveMotor      The TalonFX controller controlling the drive motor.
     * @param steeringMotor   The SparkMax controller controlling the steering
     *                        motor.
     * @param steeringEncoder The CTRE CANCoder attached to the steering motor.
     */
    public MixedSwerveModuleIO(TalonFX driveMotor, CANSparkMax steeringMotor, CANCoder steeringEncoder,
            SwerveModuleConstants constants) {
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.steeringEncoder = steeringEncoder;
        this.constants = constants;

        this.configDriveMotor();
        this.configSteeringMotor();
        this.configAngleEncoder();
        this.resetAbsolutePosition();

        // Sets the setpoint for the steering motor to the absolute position of the
        // sensor, which prevents the swerve module from instantly targeting going
        // forward at startup.
        this.steeringPID.setSetpoint(this.steeringEncoder.getAbsolutePosition());

        // Sets the PID so that if, for example, the setpoint is 10 and the measurement
        // value is 350, it's output is positive instead of negative so that it wraps
        // measurement eventually gets to 10 by wrapping around.
        this.steeringPID.enableContinuousInput(0, 360);

        // PID will stop once it's 0.3 degrees away from the setpoint.
        this.steeringPID.setTolerance(0.3);
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
    public MixedSwerveModuleIO(SwerveModuleConstants constants) {
        this(
                new TalonFX(constants.driveMotorID),
                new CANSparkMax(constants.angleMotorID, MotorType.kBrushless),
                new CANCoder(constants.cancoderID),
                constants);
    }

    private void configAngleEncoder() {
        this.steeringEncoder.configFactoryDefault();
        this.steeringEncoder.configAllSettings(CTREConfigs.swerveCanCoderConfig);
    }

    private void configDriveMotor() {
        this.driveMotor.configFactoryDefault();
        this.driveMotor.configAllSettings(CTREConfigs.swerveDriveFXConfig);
        this.driveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        this.driveMotor.setNeutralMode(Constants.Swerve.Mixed.driveNeutralMode);
        this.driveMotor.setSelectedSensorPosition(0);
    }

    private void configSteeringMotor() {
        this.steeringMotor.restoreFactoryDefaults();
        this.steeringMotor.setInverted(true);
        this.steeringMotor.setSmartCurrentLimit(Constants.Swerve.anglePeakCurrentLimit,
                Constants.Swerve.angleContinuousCurrentLimit);
        this.steeringMotor.setIdleMode(Constants.Swerve.Mixed.angleNeutralMode);
    }

    private void resetAbsolutePosition() {
        double angle = this.getAngle().getDegrees() - this.constants.angleOffset.getDegrees();
        this.steeringEncoder.setPosition(angle);
    }

    @Override
    public void periodic() {
        double power = this.steeringPID.calculate(this.steeringEncoder.getAbsolutePosition());
        if (!this.steeringPID.atSetpoint()) {
            this.steeringMotor.set(power);
        }
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
        this.steeringPID.setSetpoint(angle.getDegrees());
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.falconToMPS(this.driveMotor.getSelectedSensorVelocity(),
                        Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio),
                Rotation2d.fromDegrees(this.steeringEncoder.getAbsolutePosition()));
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.falconToMeters(this.driveMotor.getSelectedSensorPosition(),
                        Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio),
                Rotation2d.fromDegrees(this.steeringEncoder.getAbsolutePosition()));
    }

    @Override
    public void initializeShuffleBoardLayout(ShuffleboardLayout layout) {
        layout.addDouble("Angle", () -> this.getAngle().getDegrees());
        layout.addDouble("Speed", this::getSpeed);
        layout.addDouble("Position", () -> this.getPosition().distanceMeters);
        layout.addDouble("Drive Motor Stator Current", this.driveMotor::getStatorCurrent);
        layout.addDouble("Drive Motor Supply Current", this.driveMotor::getSupplyCurrent);
        layout.addDouble("Steering Motor Output Current", this.steeringMotor::getOutputCurrent);
    }
}
