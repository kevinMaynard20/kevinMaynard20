package frc.robot.commands.feedercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;

public class FeederCommand extends CommandBase {
	private final FeederSubsystem m_feederSubsystem;

	/**
	 * Initializes a new instance of the {@link FeederCommand} class.
	 * 
	 * @param feederSubsystem {@link FeederSubsystem} to be used.
	 */
	public FeederCommand(FeederSubsystem feederSubsystem) {
		m_feederSubsystem = feederSubsystem;
		addRequirements(feederSubsystem);
	}

	/**
	 * Update the motor output
	 */
	public void initialize() {
		m_feederSubsystem.setSpeed(FeederConstants.kSpeed);
	}

	/**
	 * Stop the feeder at the end of the command
	 */
	public void end(boolean interrupted) {
		m_feederSubsystem.setSpeed(0.0);
	}
}