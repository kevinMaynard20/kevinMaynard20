package frc.robot.commands.intakecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem m_intakeSubsystem;

    /**
     * Initializes a new instance of the {@link IntakeCommand} class.
     * 
     * @param intakeSubsystem {@link IntakeSubsystem} to be used.
     */
    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    /**
     * Run the motor forwards
     */
    public void initialize() {
        m_intakeSubsystem.setSpeed(IntakeConstants.kIntakeSpeed);
    }
}