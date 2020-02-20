package frc.robot.commands.climbercommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class DriveScissorsCommand extends CommandBase {

    private final ClimberSubsystem m_climberSubsystem;
    private final Supplier<Double> m_speed;

    /**
     * Drive the climber manually with percent output
     * 
     * @param climberSubsystem The climber subsystem to be used
     */
    public DriveScissorsCommand(ClimberSubsystem climberSubsystem, Supplier<Double> speed) {
        m_climberSubsystem = climberSubsystem;
        m_speed = speed;
        addRequirements(climberSubsystem);
    }

    /**
     * Update the climber motor output
     */
    public void initialize() {
        m_climberSubsystem.setPercentOutput(m_speed.get());
    }
}