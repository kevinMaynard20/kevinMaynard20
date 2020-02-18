package frc.robot.commands.shootcommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class DriveHoodCommand extends CommandBase {
    HoodSubsystem m_hoodSubsystem;

    Supplier<Double> m_speed;

    public DriveHoodCommand(HoodSubsystem hoodSubsystem, Supplier<Double> speed) {
        m_hoodSubsystem = hoodSubsystem;

        m_speed = speed;

        addRequirements(m_hoodSubsystem);
    }

    @Override
    public void execute() {
        m_hoodSubsystem.setPercentOutput(m_speed.get());
    }

    @Override
    public void end(boolean interrupted) {
        m_hoodSubsystem.resetEncoder();
    }
}