package frc.robot.testcommands;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.swerve.MixedMotorConstants;

public class TestManager extends CommandBase {
    Command activeTest;

    CANSparkMax[] driveMotors;
    TalonFX[] angleMotors;

    public TestManager() {
        driveMotors = new CANSparkMax[] {
            new CANSparkMax(MixedMotorConstants.Mod0.driveMotorID, MotorType.kBrushless),
            new CANSparkMax(MixedMotorConstants.Mod1.driveMotorID, MotorType.kBrushless),
            new CANSparkMax(MixedMotorConstants.Mod2.driveMotorID, MotorType.kBrushless),
            new CANSparkMax(MixedMotorConstants.Mod3.driveMotorID, MotorType.kBrushless),
        };

        angleMotors = new TalonFX[] {
            new TalonFX(MixedMotorConstants.Mod0.angleMotorID),
            new TalonFX(MixedMotorConstants.Mod1.angleMotorID),
            new TalonFX(MixedMotorConstants.Mod2.angleMotorID),
            new TalonFX(MixedMotorConstants.Mod3.angleMotorID),
        };
    }

    @Override
    public void initialize() {
        if (this.activeTest != null) {
            this.activeTest.cancel();
        }

        this.activeTest = new NeoTester(this.driveMotors[0]);
        this.activeTest.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        if (this.activeTest != null) {
            this.activeTest.cancel();
            this.activeTest = null;
        }
    }
}
