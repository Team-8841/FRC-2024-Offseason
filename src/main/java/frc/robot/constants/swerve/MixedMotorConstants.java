package frc.robot.constants.swerve;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.SwerveModuleConstants;


public class MixedMotorConstants {
    public static final TalonFXConfiguration driveMotorConfigs = new TalonFXConfiguration();

    /* Angle Motor PID Values */
    public static final double angleKP = SwerveConstants.chosenModule.angleKP;
    public static final double angleKI = SwerveConstants.chosenModule.angleKI;
    public static final double angleKD = SwerveConstants.chosenModule.angleKD;
    public static final double angleKF = SwerveConstants.chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.010009775171065494; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;

    /*
     * Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE
     */
    // TODO: Check if these need to be converted.
    public static final double driveKS = (0.32 / 12); 
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kCoast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { 
        public static final int driveMotorID = 1;
        public static final int angleMotorID = 2;
        public static final int canCoderID = 12;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                angleMotorID, canCoderID, angleOffset, 0);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1  {
        public static final int driveMotorID = 3;
        public static final int angleMotorID = 4;
        public static final int canCoderID = 5;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                angleMotorID, canCoderID, angleOffset, 1);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { 
        public static final int driveMotorID = 6;
        public static final int angleMotorID = 7;
        public static final int canCoderID = 8;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                angleMotorID, canCoderID, angleOffset, 2);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
        public static final int driveMotorID = 9;
        public static final int angleMotorID = 10;
        public static final int canCoderID = 11;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                angleMotorID, canCoderID, angleOffset, 3);
    }

    static {
        /* Drive motor */

        Slot0Configs drivePIDConfigs = new Slot0Configs();
        drivePIDConfigs.kP = driveKP;
        drivePIDConfigs.kI = driveKI;
        drivePIDConfigs.kD = driveKD;
        drivePIDConfigs.kS = driveKS;
        drivePIDConfigs.kV = driveKV;
        driveMotorConfigs.Slot0 = drivePIDConfigs;

        CurrentLimitsConfigs driveCurrentLimit = new CurrentLimitsConfigs();
        driveCurrentLimit.SupplyCurrentLimitEnable = SwerveConstants.driveEnableCurrentLimit;
        driveCurrentLimit.SupplyCurrentLimit = SwerveConstants.driveContinuousCurrentLimit;
        driveCurrentLimit.SupplyCurrentThreshold = SwerveConstants.drivePeakCurrentLimit;
        driveCurrentLimit.SupplyTimeThreshold = SwerveConstants.drivePeakCurrentDuration;
        driveMotorConfigs.CurrentLimits = driveCurrentLimit;

        OpenLoopRampsConfigs driveOpenRampConfigs = new OpenLoopRampsConfigs();
        driveOpenRampConfigs.VoltageOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
        driveMotorConfigs.OpenLoopRamps = driveOpenRampConfigs;

        ClosedLoopRampsConfigs driveClosedRampConfigs = new ClosedLoopRampsConfigs();
        driveClosedRampConfigs.VoltageClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;
        driveMotorConfigs.ClosedLoopRamps = driveClosedRampConfigs;

        MotorOutputConfigs driveMotorOutConfigs = new MotorOutputConfigs();
        driveMotorOutConfigs.Inverted = SwerveConstants.driveMotorInvert;
        driveMotorOutConfigs.NeutralMode = driveNeutralMode;
        driveMotorConfigs.MotorOutput = driveMotorOutConfigs;
    }
}
