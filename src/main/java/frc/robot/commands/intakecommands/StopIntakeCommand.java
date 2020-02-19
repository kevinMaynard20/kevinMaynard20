package frc.robot.commands.intakecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntakeCommand extends CommandBase {

    private final IntakeSubsystem m_intakeSubsystem;

    /**
     * Initializes a new instance of the {@link IntakeCommand} class.
     * 
     * @param intakeSubsystem {@link IntakeSubsystem} to be used.
     */
    public StopIntakeCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    /**
     * Stop the motor
     */
    public void initialize() {
        m_intakeSubsystem.setSpeed(0);
    }
}