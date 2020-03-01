package frc.robot.commands.carouselcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CarouselConstants;
import frc.robot.subsystems.CarouselSubsystem;

public class ResetCarouselCommand extends CommandBase {

    private final CarouselSubsystem m_carouselSubsystem;

    /**
     * Run the carousel at normal speed
     * 
     * @param carouselSubsystem {@link CarouselSubsystem} to be used.
     */
    public ResetCarouselCommand(CarouselSubsystem carouselSubsystem) {
        m_carouselSubsystem = carouselSubsystem;
        addRequirements(carouselSubsystem);
    }

    /**
     * Run the carousel
     */
    public void initialize() {
        if (m_carouselSubsystem.atOpenSpace())
            m_carouselSubsystem.setVelocity(CarouselConstants.kVelocity * CarouselConstants.kRatio);
        else
            m_carouselSubsystem.setVelocity(0);
    }

    /**
     * Stop the carousel at the end of the command
     */
    public void end(boolean interrupted) {
        m_carouselSubsystem.setVelocity(0.0);
    }
}