package frc.robot.commands.carouselcommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CarouselConstants;
import frc.robot.subsystems.CarouselSubsystem;

public class ToOpenSpaceCommand extends CommandBase {

    private final CarouselSubsystem m_carouselSubsystem;
    private final PIDController m_pidController = new PIDController(500, .2, .1);

    /**
     * Set the carousel to be ready for shooting whenever it is not running
     * 
     * @param carouselSubsystem {@link CarouselSubsystem} to be used.
     */
    public ToOpenSpaceCommand(CarouselSubsystem carouselSubsystem) {
        m_carouselSubsystem = carouselSubsystem;
        addRequirements(m_carouselSubsystem);
    }

    public void initialize() {
        m_pidController.setSetpoint(
                Math.ceil(m_carouselSubsystem.getPosition() / CarouselConstants.kRatio) * CarouselConstants.kRatio);
    }

    public void execute() {
        double speed = m_pidController.calculate(m_carouselSubsystem.getPosition());
        speed = speed > 20 * CarouselConstants.kRatio ? 20 * CarouselConstants.kRatio : speed;
        m_carouselSubsystem.setVelocity(speed);
    }

    /**
     * Stop the carousel at the end of the command
     */
    public void end(boolean interrupted) {
        m_carouselSubsystem.setVelocity(0.0);
    }
}