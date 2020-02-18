package frc.robot.commands.shootcommands;

import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootCommand extends CommandBase {

    private final FlywheelSubsystem m_flywheelSubsystem;
    private final HoodSubsystem m_hoodSubsystem;
    private final double m_flywheelSetpoint;

    public ShootCommand(FlywheelSubsystem flywheelSubsystem, HoodSubsystem hoodSubsystem, double flywheelSetpoint) {
        m_flywheelSubsystem = flywheelSubsystem;
        m_hoodSubsystem = hoodSubsystem;
        m_flywheelSetpoint = flywheelSetpoint;

    }

    public void initialize() {
        m_flywheelSubsystem.setSetpoint(m_flywheelSetpoint);
        m_hoodSubsystem.setPosition(calculateHoodSetpoint());
    }

    public double calculateHoodSetpoint() {
        // TODO - Kevin do the math here
        return 0;
    }

    public void end(boolean interrupted) {
        m_flywheelSubsystem.setSetpoint(0);
    }
}
