package frc.robot.commands.carouselcommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldLocation;
import frc.robot.subsystems.CarouselSubsystem;

public class AutoSpeedCarouselCommand extends CommandBase {

    private final CarouselSubsystem m_carouselSubsystem;
    private final Supplier<Double> m_flywheelSetpoint;

    /**
     * Run the carousel at speed determined by flywheel
     * 
     * @param carouselSubsystem {@link CarouselSubsystem} to be used.
     */
    public AutoSpeedCarouselCommand(CarouselSubsystem carouselSubsystem, Supplier<Double> flywheelSetpoint) {
        m_carouselSubsystem = carouselSubsystem;
        m_flywheelSetpoint = flywheelSetpoint;
        addRequirements(m_carouselSubsystem);
    }

    public void execute() {
        m_carouselSubsystem.setVelocity(FieldLocation.fromFlywheelSetpoint(m_flywheelSetpoint.get()).carouselSetpoint);
    }

    /**
     * Stop the carousel at the end of the command
     */
    public void end(boolean interrupted) {
        m_carouselSubsystem.setVelocity(0.0);
    }
}