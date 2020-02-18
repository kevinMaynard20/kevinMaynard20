package frc.robot.commands.shootcommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class HoodPositionCommand extends CommandBase {
    HoodSubsystem m_hoodSubsystem;

    Supplier<Long> m_speed;

    public HoodPositionCommand(HoodSubsystem hoodSubsystem, Supplier<Long> speed) {
        m_hoodSubsystem = hoodSubsystem;

        m_speed = speed;

        addRequirements(m_hoodSubsystem);
    }

    @Override
    public void execute() {
        m_hoodSubsystem.setSetPoint(m_speed.get());
    }
}