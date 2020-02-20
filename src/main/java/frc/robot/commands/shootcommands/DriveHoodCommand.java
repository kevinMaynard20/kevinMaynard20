package frc.robot.commands.shootcommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class DriveHoodCommand extends CommandBase {

    private final HoodSubsystem m_hoodSubsystem;
    private final Supplier<Double> m_speed;

    /**
     * Drive the hood using percent output
     * 
     * @param hoodSubsystem The hood subsystem to be used
     * @param speed         Percent output supplier
     */
    public DriveHoodCommand(HoodSubsystem hoodSubsystem, Supplier<Double> speed) {
        m_hoodSubsystem = hoodSubsystem;
        m_speed = speed;
        addRequirements(m_hoodSubsystem);
    }

    /**
     * Update the motor output
     */
    public void execute() {
        m_hoodSubsystem.setPercentOutput(m_speed.get());
    }

    /**
     * Set the setposition to the current position when the command ends
     */
    public void end(boolean interrupted) {
        m_hoodSubsystem.setSetpoint(m_hoodSubsystem.getPosition());
    }
}