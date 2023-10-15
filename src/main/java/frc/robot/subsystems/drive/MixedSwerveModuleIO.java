package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.constants.swerve.MixedMotorConstants;
import frc.robot.constants.swerve.SwerveConstants;

public class MixedSwerveModuleIO implements SwerveModuleIO {
    private TalonFX driveMotor;
    private CANSparkMax steeringMotor;
    private CANcoder steeringEncoder;

    private Rotation2d lastAngle;

    // I would've liked to use closed loop control on the neo but the cancoder can't
    // be attached to the sparkmax :/
    PIDController steeringPID = new PIDController(MixedMotorConstants.angleKP, MixedMotorConstants.angleKI,
                MixedMotorConstants.angleKD);

    VelocityVoltage driveVelVoltage = new VelocityVoltage(0).withSlot(0);

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
    public MixedSwerveModuleIO(TalonFX driveMotor, CANSparkMax steeringMotor, CANcoder steeringEncoder,
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
        this.steeringPID.setSetpoint(this.steeringEncoder.getAbsolutePosition().getValue());

        // Sets the PID so that if, for example, the setpoint is 10 and the measurement
        // value is 350, it's output is positive instead of negative so that it wraps
        // measurement eventually gets to 10 by wrapping around.
        this.steeringPID.enableContinuousInput(0, 360);

        // PID will stop once it's 0.3 degrees away from the setpoint.
        this.steeringPID.setTolerance(0.3);
    }

    /**
     * Creates a new container from each of the components.
     * 
     * @param constants Module specific constants.
     */
    public MixedSwerveModuleIO(SwerveModuleConstants constants) {
        this(
                new TalonFX(constants.driveMotorID),
                new CANSparkMax(constants.angleMotorID, MotorType.kBrushless),
                new CANcoder(constants.cancoderID),
                constants);
    }

    private void configAngleEncoder() {
        CANcoderConfigurator configurator = this.steeringEncoder.getConfigurator();
        CANcoderConfiguration configuration = SwerveConstants.canCoderConfigs;
        configuration.MagnetSensor.MagnetOffset = -this.constants.angleOffset.getRotations();
        configurator.apply(configuration);
    }

    private void configDriveMotor() {
        this.driveMotor.getConfigurator().apply(MixedMotorConstants.driveMotorConfigs);
    }

    private void configSteeringMotor() {
        this.steeringMotor.restoreFactoryDefaults();
        this.steeringMotor.setInverted(SwerveConstants.angleMotorInvert == InvertedValue.Clockwise_Positive);
        this.steeringMotor.setSmartCurrentLimit(SwerveConstants.anglePeakCurrentLimit,
                SwerveConstants.angleContinuousCurrentLimit);
        this.steeringMotor.setIdleMode(MixedMotorConstants.angleNeutralMode);
    }

    private void resetAbsolutePosition() {
        double angle = this.getAngle().getDegrees() - this.constants.angleOffset.getDegrees();
        this.steeringEncoder.setPosition(angle);
    }

    @Override
    public void periodic() {
        double power = this.steeringPID.calculate(this.steeringEncoder.getAbsolutePosition().getValue());
        if (!this.steeringPID.atSetpoint()) {
            this.steeringMotor.set(power);
        }
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState) {
        // Closed loop
        double velocity = Conversions.metersToRots(desiredState.speedMetersPerSecond,
                SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio);
        this.driveMotor.setControl(this.driveVelVoltage.withVelocity(velocity));
    }

    @Override
    public void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;

        this.lastAngle = angle;
        this.steeringPID.setSetpoint(angle.getDegrees());
    }

    @Override
    public SwerveModuleState getState() {
        double angle = this.steeringEncoder.getAbsolutePosition().getValue();

        return new SwerveModuleState(
                Conversions.rotsToMeters(this.driveMotor.getVelocity().getValue(),
                        SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio),
                Rotation2d.fromRotations(angle));
    }

    @Override
    public SwerveModulePosition getPosition() {
        double angle = this.steeringEncoder.getAbsolutePosition().getValue();

        return new SwerveModulePosition(
                Conversions.rotsToMeters(this.driveMotor.getPosition().getValue(),
                        SwerveConstants.wheelCircumference, SwerveConstants.driveGearRatio),
                Rotation2d.fromRotations(angle));
    }

    @Override
    public void initializeShuffleBoardLayout(ShuffleboardLayout layout) {
        layout.addDouble("Angle", () -> this.getAngle().getDegrees());
        layout.addDouble("Speed", this::getSpeed);
        layout.addDouble("Position", () -> this.getPosition().distanceMeters);
        /*
        layout.addDouble("Drive Motor Stator Current", this.driveMotor::getStatorCurrent);
        layout.addDouble("Drive Motor Supply Current", this.driveMotor::getSupplyCurrent);
        layout.addDouble("Steering Motor Output Current", this.steeringMotor::getOutputCurrent);
        */
    }
}
