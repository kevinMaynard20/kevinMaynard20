package frc.robot.commands.carouselcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CarouselConstants;
import frc.robot.subsystems.CarouselSubsystem;

public class FasterCarouselCommand extends CommandBase {

    private final CarouselSubsystem m_carouselSubsystem;

    /**
     * Initializes a new instance of the {@link FasterCarouselCommand} class.
     * 
     * @param carouselSubsystem {@link CarouselSubsystem} to be used.
     */
    public FasterCarouselCommand(CarouselSubsystem carouselSubsystem) {
        m_carouselSubsystem = carouselSubsystem;
        addRequirements(m_carouselSubsystem);
    }

    /**
     * Run the carousel
     */
    public void initialize() {
        m_carouselSubsystem.setVelocity(CarouselConstants.kIntakeVelocity * CarouselConstants.kRatio);
    }

    /**
     * Stop the carousel at the end of the command
     */
    public void end(boolean interrupted) {
        m_carouselSubsystem.setVelocity(0.0);
    }
}