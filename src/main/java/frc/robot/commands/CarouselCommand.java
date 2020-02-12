package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CarouselConstants;
import frc.robot.subsystems.CarouselSubsystem;

public class CarouselCommand extends CommandBase {
	private final CarouselSubsystem m_carouselSubsystem;

	/**
	 * Initializes a new instance of the {@link CarouselCommand} class.
	 * 
	 * @param carouselSubsystem {@link CarouselSubsystem} to be used.
	 */
	public CarouselCommand(CarouselSubsystem carouselSubsystem) {
		m_carouselSubsystem = carouselSubsystem;

		addRequirements(m_carouselSubsystem);
	}

	@Override
	public void initialize() {
		m_carouselSubsystem.setVelocity(CarouselConstants.kVelocity * CarouselConstants.kRatio);
	}

	@Override
	public void end(boolean interrupted) {
		m_carouselSubsystem.setVelocity(0.0);
	}
}