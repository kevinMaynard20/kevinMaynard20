package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;

public class FeederCommand extends CommandBase {
	FeederSubsystem m_feederSubsystem;

	/**
	 * Initializes a new instance of the {@link FeederCommand} class.
	 * 
	 * @param feederSubsystem {@link FeederSubsystem} to be used.
	 */
	public FeederCommand(FeederSubsystem feederSubsystem) {
		m_feederSubsystem = feederSubsystem;

		addRequirements(m_feederSubsystem);
	}

	@Override
	public void execute() {
		m_feederSubsystem.setSpeed(FeederConstants.kSpeed);
	}

	@Override
	public void end(boolean interrupted) {
		m_feederSubsystem.setSpeed(0.0);
	}
}