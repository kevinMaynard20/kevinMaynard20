package frc.robot.commands.shootcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;

public class ShootCommand extends CommandBase {

    private final FlywheelSubsystem m_flywheelSubsystem;
    private final HoodSubsystem m_hoodSubsystem;
    private final double m_flywheelSetpoint, m_hoodSetpoint;

    /**
     * 
     * @param flywheelSubsystem The flywheel subsystem to be used
     * @param hoodSubsystem     The hood subsystem to be used
     * @param flywheelSetpoint  The flywheel setpoint
     * @param hoodSetpoint      The hood setpoint
     */
    public ShootCommand(FlywheelSubsystem flywheelSubsystem, HoodSubsystem hoodSubsystem, double flywheelSetpoint,
            double hoodSetpoint) {
        m_flywheelSubsystem = flywheelSubsystem;
        m_hoodSubsystem = hoodSubsystem;
        m_flywheelSetpoint = flywheelSetpoint;
        m_hoodSetpoint = hoodSetpoint;
        addRequirements(flywheelSubsystem, hoodSubsystem);
    }

    /**
     * Set the setpoints of the flywheel and hood
     */
    public void initialize() {
        m_flywheelSubsystem.setSetpoint(m_flywheelSetpoint);
        m_hoodSubsystem.setSetPoint(m_hoodSetpoint);
    }
}
