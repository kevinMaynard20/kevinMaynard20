package frc.robot.commands.shootcommands;

import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightShootCommand extends CommandBase {

    private final FlywheelSubsystem m_flywheelSubsystem;
    private final HoodSubsystem m_hoodSubsystem;
    private final LimelightSubsystem m_limelightSubsystem;
    private double m_flywheelSetpoint;

    public LimelightShootCommand(FlywheelSubsystem flywheelSubsystem, HoodSubsystem hoodSubsystem,
            LimelightSubsystem limelightSubsystem) {
        m_flywheelSubsystem = flywheelSubsystem;
        m_hoodSubsystem = hoodSubsystem;
        m_limelightSubsystem = limelightSubsystem;
        addRequirements(flywheelSubsystem, m_hoodSubsystem);
    }

    public void initialize() {
        m_flywheelSetpoint = calculateFlywheelSetpoint();
        m_flywheelSubsystem.setSetpoint(m_flywheelSetpoint);
        m_hoodSubsystem.setSetPoint(calculateHoodSetpoint());
    }

    public double calculateFlywheelSetpoint() {
        // TODO - Kevin do the math here
        return m_limelightSubsystem.getDistance();
    }

    public double calculateHoodSetpoint() {
        // TODO - Kevin do the math here
        return 0;
    }

    public void end(boolean interrupted) {
        m_flywheelSubsystem.setSetpoint(0);
    }
}
