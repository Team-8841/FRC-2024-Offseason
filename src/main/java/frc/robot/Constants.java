// Shallow graves to all dead rats (i like the dark clouds the best)

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * Various constants used throughout the program are defined here. They're
 * defined here instead of elsewhere just for ease of changing them.
 */
public final class Constants {
    public static final boolean simReplay = false;
    public static final double controllerDeadband = 0.1;

    /* Copied straight from BaseFalconSwerve */

    public static final class Swerve {
        public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants
                .SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73); // TODO: This must be tuned to specific
                                                                             // robot
        public static final double wheelBase = Units.inchesToMeters(21.73); // TODO: This must be tuned to specific
                                                                            // robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue canCoderDir = chosenModule.canCoderDir;

        /* Swerve Current Limiting */
        /*
         * When using the NEO, only *PeakCurrentLimit and *ContinousCurrentLimit are
         * used
         */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 3; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 3; // TODO: This must be tuned to specific robot

        /* Motor/encoder specific */

        public static final CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();

        static {
            MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
            magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
            magnetSensorConfigs.SensorDirection = Constants.Swerve.canCoderDir;
        }

        public static final class PureTalonFX {
            public static final TalonFXConfiguration angleMotorConfigs = new TalonFXConfiguration();
            public static final TalonFXConfiguration driveMotorConfigs = new TalonFXConfiguration();

            /* Angle Motor PID Values */
            public static final double angleKP = chosenModule.angleKP;
            public static final double angleKI = chosenModule.angleKI;
            public static final double angleKD = chosenModule.angleKD;
            public static final double angleKF = chosenModule.angleKF;

            /* Drive Motor PID Values */
            public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
            public static final double driveKI = 0.0;
            public static final double driveKD = 0.0;

            /*
             * Drive Motor Characterization Values
             * Divide SYSID values by 12 to convert from volts to percent output for CTRE
             */
            public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
            public static final double driveKV = (1.51 / 12);
            public static final double driveKA = (0.27 / 12); // (unused)

            /* Neutral Modes */
            public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
            public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

            /* Module Specific Constants */
            /* Front Left Module - Module 0 */
            public static final class Mod0 { 
                public static final int driveMotorID = 1;
                public static final int angleMotorID = 2;
                public static final int canCoderID = 1;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                        angleMotorID, canCoderID, angleOffset, 0);
            }

            /* Front Right Module - Module 1 */
            public static final class Mod1 { 
                public static final int driveMotorID = 3;
                public static final int angleMotorID = 4;
                public static final int canCoderID = 2;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                        angleMotorID, canCoderID, angleOffset, 1);
            }

            /* Back Left Module - Module 2 */
            public static final class Mod2 { 
                public static final int driveMotorID = 5;
                public static final int angleMotorID = 6;
                public static final int canCoderID = 3;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                        angleMotorID, canCoderID, angleOffset, 2);
            }

            /* Back Right Module - Module 3 */
            public static final class Mod3 { 
                public static final int driveMotorID = 7;
                public static final int angleMotorID = 8;
                public static final int canCoderID = 4;
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
                driveCurrentLimit.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
                driveCurrentLimit.SupplyCurrentLimit = Constants.Swerve.driveContinuousCurrentLimit;
                driveCurrentLimit.SupplyCurrentThreshold = Constants.Swerve.drivePeakCurrentLimit;
                driveCurrentLimit.SupplyTimeThreshold = Constants.Swerve.drivePeakCurrentDuration;
                driveMotorConfigs.CurrentLimits = driveCurrentLimit;

                OpenLoopRampsConfigs driveOpenRampConfigs = new OpenLoopRampsConfigs();
                driveOpenRampConfigs.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
                driveMotorConfigs.OpenLoopRamps = driveOpenRampConfigs;

                ClosedLoopRampsConfigs driveClosedRampConfigs = new ClosedLoopRampsConfigs();
                driveClosedRampConfigs.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
                driveMotorConfigs.ClosedLoopRamps = driveClosedRampConfigs;

                MotorOutputConfigs driveMotorOutConfigs = new MotorOutputConfigs();
                driveMotorOutConfigs.Inverted = Constants.Swerve.driveMotorInvert;
                driveMotorOutConfigs.NeutralMode = Constants.Swerve.PureTalonFX.driveNeutralMode;
                driveMotorConfigs.MotorOutput = driveMotorOutConfigs;

                /* Angle motor */

                Slot0Configs anglePIDConfigs = new Slot0Configs();
                anglePIDConfigs.kP = angleKP;
                anglePIDConfigs.kI = angleKI;
                anglePIDConfigs.kD = angleKD;
                anglePIDConfigs.kV = angleKF;
                angleMotorConfigs.Slot0 = anglePIDConfigs;

                CurrentLimitsConfigs angleCurrentConfigs = new CurrentLimitsConfigs();
                angleCurrentConfigs.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
                angleCurrentConfigs.SupplyCurrentLimit = Constants.Swerve.angleContinuousCurrentLimit;
                angleCurrentConfigs.SupplyCurrentThreshold = Constants.Swerve.anglePeakCurrentLimit;
                angleCurrentConfigs.SupplyTimeThreshold = Constants.Swerve.anglePeakCurrentDuration;
                angleMotorConfigs.CurrentLimits = angleCurrentConfigs;

                ClosedLoopGeneralConfigs angleClosedGeneralConfigs = new ClosedLoopGeneralConfigs();
                angleClosedGeneralConfigs.ContinuousWrap = true;
                angleMotorConfigs.ClosedLoopGeneral = angleClosedGeneralConfigs;

                MotorOutputConfigs angleMotorOutConfigs = new MotorOutputConfigs();
                angleMotorOutConfigs.Inverted = Constants.Swerve.angleMotorInvert;
                angleMotorOutConfigs.NeutralMode = Constants.Swerve.PureTalonFX.angleNeutralMode;
                angleMotorConfigs.MotorOutput = angleMotorOutConfigs;
            }
        }

        public static final class Mixed {
            public static final TalonFXConfiguration driveMotorConfigs = new TalonFXConfiguration();

            /* Angle Motor PID Values */
            public static final double angleKP = chosenModule.angleKP;
            public static final double angleKI = chosenModule.angleKI;
            public static final double angleKD = chosenModule.angleKD;
            public static final double angleKF = chosenModule.angleKF;

            /* Drive Motor PID Values */
            public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
            public static final double driveKI = 0.0;
            public static final double driveKD = 0.0;

            /*
             * Drive Motor Characterization Values
             * Divide SYSID values by 12 to convert from volts to percent output for CTRE
             */
            public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
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
                public static final int canCoderID = 1;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                        angleMotorID, canCoderID, angleOffset, 0);
            }

            /* Front Right Module - Module 1 */
            public static final class Mod1  {
                public static final int driveMotorID = 3;
                public static final int angleMotorID = 4;
                public static final int canCoderID = 2;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                        angleMotorID, canCoderID, angleOffset, 1);
            }

            /* Back Left Module - Module 2 */
            public static final class Mod2 { 
                public static final int driveMotorID = 5;
                public static final int angleMotorID = 6;
                public static final int canCoderID = 3;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                        angleMotorID, canCoderID, angleOffset, 2);
            }

            /* Back Right Module - Module 3 */
            public static final class Mod3 {
                public static final int driveMotorID = 7;
                public static final int angleMotorID = 8;
                public static final int canCoderID = 4;
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
                driveCurrentLimit.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
                driveCurrentLimit.SupplyCurrentLimit = Constants.Swerve.driveContinuousCurrentLimit;
                driveCurrentLimit.SupplyCurrentThreshold = Constants.Swerve.drivePeakCurrentLimit;
                driveCurrentLimit.SupplyTimeThreshold = Constants.Swerve.drivePeakCurrentDuration;
                driveMotorConfigs.CurrentLimits = driveCurrentLimit;

                OpenLoopRampsConfigs driveOpenRampConfigs = new OpenLoopRampsConfigs();
                driveOpenRampConfigs.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
                driveMotorConfigs.OpenLoopRamps = driveOpenRampConfigs;

                ClosedLoopRampsConfigs driveClosedRampConfigs = new ClosedLoopRampsConfigs();
                driveClosedRampConfigs.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
                driveMotorConfigs.ClosedLoopRamps = driveClosedRampConfigs;

                MotorOutputConfigs driveMotorOutConfigs = new MotorOutputConfigs();
                driveMotorOutConfigs.Inverted = Constants.Swerve.driveMotorInvert;
                driveMotorOutConfigs.NeutralMode = Constants.Swerve.PureTalonFX.driveNeutralMode;
                driveMotorConfigs.MotorOutput = driveMotorOutConfigs;
            }
        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double MaxSpeedMetersPerSecond = 3;
        public static final double MaxAccelerationMetersPerSecondSquared = 3;
        public static final double MaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double MaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double PXController = 1;
        public static final double PYController = 1;
        public static final double PThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                MaxAngularSpeedRadiansPerSecond, MaxAngularSpeedRadiansPerSecondSquared);
    }
}
