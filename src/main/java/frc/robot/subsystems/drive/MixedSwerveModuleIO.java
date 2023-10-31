package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.constants.swerve.MixedMotorConstants;
import frc.robot.constants.swerve.SwerveConstants;

public class MixedSwerveModuleIO implements SwerveModuleIO {
    private CANSparkMax driveMotor;
    private TalonFX steeringMotor;
    private CANcoder steeringEncoder;

    private SparkMaxPIDController sparkMaxPID;

    private Rotation2d lastAngle;

    PositionDutyCycle driveVelDutyCycle = new PositionDutyCycle(0).withSlot(0);
    SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(MixedMotorConstants.driveKS, MixedMotorConstants.driveKV, MixedMotorConstants.driveKA);

    SwerveModuleConstants constants;

    /**
     * Creates a new container from each of the components.
     * 
     * @param driveMotor      The TalonFX controller controlling the drive motor.
     * @param steeringMotor   The SparkMax controller controlling the steering
     *                        motor.
     * @param steeringEncoder The CTRE CANCoder attached to the steering motor.
     * @param constants Module specific constants.
     */
    public MixedSwerveModuleIO(CANSparkMax driveMotor, TalonFX steeringMotor, CANcoder steeringEncoder,
            SwerveModuleConstants constants) {
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.steeringEncoder = steeringEncoder;
        this.constants = constants;

        this.configDriveMotor();
        this.configSteeringMotor();
        this.configSteeringEncoder();

        // Sets the setpoint for the steering motor to the absolute position of the
        // sensor, which prevents the swerve module from instantly targeting going
        // forward at startup.
        // this.steeringPID.setSetpoint(this.steeringEncoder.getAbsolutePosition().getValue());

        // Sets the PID so that if, for example, the setpoint is 10 and the measurement
        // value is 350, it's output is positive instead of negative so that it wraps
        // measurement eventually gets to 10 by wrapping around.
        // this.steeringPID.enableContinuousInput(0, 360);

        // PID will stop once it's 0.3 degrees away from the setpoint.
        // this.steeringPID.setTolerance(0.3);
    }

    /**
     * Creates a new container from each of the components.
     * 
     * @param constants Module specific constants.
     */
    public MixedSwerveModuleIO(SwerveModuleConstants constants) {
        this(
                new CANSparkMax(constants.angleMotorID, MotorType.kBrushless),
                new TalonFX(constants.driveMotorID),
                new CANcoder(constants.cancoderID),
                constants);
    }

    private void configSteeringEncoder() {
        CANcoderConfigurator configurator = this.steeringEncoder.getConfigurator();
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetSensorConfigs.SensorDirection = SwerveConstants.canCoderDir;
        magnetSensorConfigs.MagnetOffset = this.constants.angleOffset.getRotations();

        configurator.apply(magnetSensorConfigs);
    }

    private void configDriveMotor() {
        this.driveMotor.restoreFactoryDefaults();
        this.driveMotor.setInverted(SwerveConstants.driveMotorInvert == InvertedValue.Clockwise_Positive);
        this.driveMotor.setSmartCurrentLimit(SwerveConstants.drivePeakCurrentLimit,
                SwerveConstants.driveContinuousCurrentLimit);
        //this.driveMotor.setIdleMode(.angleNeutralMode);
        this.driveMotor.setIdleMode(MixedMotorConstants.driveNeutralMode);

        this.sparkMaxPID = this.driveMotor.getPIDController();
        this.sparkMaxPID.setP(MixedMotorConstants.driveKP);
        this.sparkMaxPID.setI(MixedMotorConstants.driveKI);
        this.sparkMaxPID.setD(MixedMotorConstants.driveKD);
        this.sparkMaxPID.setFF(0);
        this.sparkMaxPID.setFeedbackDevice(this.driveMotor.getEncoder());
    }

    private void configSteeringMotor() {
        TalonFXConfigurator configurator = this.steeringMotor.getConfigurator();
        configurator.apply(MixedMotorConstants.angleMotorConfigs);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.FeedbackRemoteSensorID = this.constants.cancoderID;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        feedbackConfigs.RotorToSensorRatio = SwerveConstants.angleGearRatio;
        configurator.apply(feedbackConfigs);
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState) {
        // Closed loop
        /* 
        double velocity = Conversions.metersToRots(desiredState.speedMetersPerSecond,
                SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio);
        this.driveMotor.setControl(this.driveVelDutyCycle.withVelocity(velocity));
        */

        double velocity = Conversions.metersToRots(desiredState.speedMetersPerSecond,
                SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio);
        double ff = this.driveFeedforward.calculate(desiredState.speedMetersPerSecond);
        this.sparkMaxPID.setReference(velocity * 60, ControlType.kVelocity, 0, ff);
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;
        
        this.lastAngle = angle;
        this.steeringMotor.setControl(this.driveVelDutyCycle.withPosition(angle.getRotations()));
    }

    @Override
    public SwerveModuleState getState() {
        double angle = this.steeringEncoder.getAbsolutePosition().getValue();

        return new SwerveModuleState(
                Conversions.rotsToMeters(this.driveMotor.getEncoder().getVelocity() / 60,
                        SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio),
                Rotation2d.fromRotations(angle));
    }

    @Override
    public SwerveModulePosition getPosition() {
        double angle = this.steeringEncoder.getAbsolutePosition().getValue();

        return new SwerveModulePosition(
                Conversions.rotsToMeters(this.driveMotor.getEncoder().getPosition(),
                        SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio),
                Rotation2d.fromRotations(angle));
    }

    @Override
    public void initializeShuffleBoardLayout(ShuffleboardLayout layout) {
        layout.addDouble("Angle", () -> this.getAngle().getDegrees());
        layout.addDouble("Speed", this::getSpeed);
        layout.addDouble("Position", () -> this.getPosition().distanceMeters);
        //layout.addDouble("Drive Motor Stator Current", () -> this.driveMotor.getStatorCurrent().getValue());
        //layout.addDouble("Drive Motor Supply Current", () -> this.driveMotor.getSupplyCurrent().getValue());
        //layout.addDouble("Steering Motor Output Current", this.steeringMotor::getOutputCurrent);
    }
}
