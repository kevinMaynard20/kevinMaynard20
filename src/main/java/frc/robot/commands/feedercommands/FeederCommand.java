package frc.robot.commands.feedercommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CarouselConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;

public class FeederCommand extends CommandBase {
	private final FeederSubsystem m_feederSubsystem;
	private final Supplier<Double> m_carouselPosition;
	private boolean started;

	/**
	 * Initializes a new instance of the {@link FeederCommand} class.
	 * 
	 * @param feederSubsystem {@link FeederSubsystem} to be used.
	 */
	public FeederCommand(FeederSubsystem feederSubsystem, Supplier<Double> carouselPosition) {
		m_feederSubsystem = feederSubsystem;
		m_carouselPosition = carouselPosition;
		addRequirements(m_feederSubsystem);
	}

	public void initialize() {
		started = false;
	}

	public void execute() {
		if (!started && (m_carouselPosition.get() % CarouselConstants.kRatio < 0.1)
				|| (m_carouselPosition.get() % CarouselConstants.kRatio > 0.9))
			started = true;
		if (started)
			m_feederSubsystem.setSpeed(FeederConstants.kSpeed);
	}

	public void end(boolean interrupted) {
		m_feederSubsystem.setSpeed(0.0);
		started = false;
	}
}