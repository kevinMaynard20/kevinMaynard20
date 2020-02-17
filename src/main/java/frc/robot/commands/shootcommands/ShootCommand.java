package frc.robot.commands.shootcommands;

import frc.robot.subsystems.FlywheelSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootCommand extends CommandBase {

    private final FlywheelSubsystem m_flywheelSubsystem;
    private final double m_setPoint;

    public ShootCommand(FlywheelSubsystem flywheelSubsystem, double setPoint) {
        m_flywheelSubsystem = flywheelSubsystem;
        m_setPoint = setPoint;
        addRequirements(flywheelSubsystem);
    }

    public void initialize() {
        m_flywheelSubsystem.setSetpoint(m_setPoint);
    }

    public void end(boolean interrupted) {
        m_flywheelSubsystem.setSetpoint(0);
    }
}
