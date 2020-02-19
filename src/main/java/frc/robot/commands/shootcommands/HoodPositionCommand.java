package frc.robot.commands.shootcommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class HoodPositionCommand extends CommandBase {

    private final HoodSubsystem m_hoodSubsystem;
    private final Supplier<Double> m_speed;

    /**
     * Drive the hood using setpoints
     * 
     * @param hoodSubsystem The hood subsystem to be used
     * @param speed         The desired encoder position
     */
    public HoodPositionCommand(HoodSubsystem hoodSubsystem, Supplier<Double> speed) {
        m_hoodSubsystem = hoodSubsystem;
        m_speed = speed;
        addRequirements(m_hoodSubsystem);
    }

    /**
     * Update the setpoint
     */
    public void execute() {
        m_hoodSubsystem.setSetPoint(m_speed.get());
    }
}