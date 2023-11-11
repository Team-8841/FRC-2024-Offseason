package frc.robot.testcommands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class NeoTester extends CommandBase {
    CANSparkMax motor;

    public NeoTester(CANSparkMax motor) {
        this.motor = motor;
    }

    public void initialize() {
        this.motor.set(0.1);
    }

    public void end(boolean interrupted) {
        this.motor.set(0);
    }
}