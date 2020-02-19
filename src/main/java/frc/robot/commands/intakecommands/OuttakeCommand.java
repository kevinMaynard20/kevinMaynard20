package frc.robot.commands.intakecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeCommand extends CommandBase {

    private final IntakeSubsystem m_intakeSubsystem;

    /**
     * Initializes a new instance of the {@link IntakeCommand} class.
     * 
     * @param intakeSubsystem {@link IntakeSubsystem} to be used.
     */
    public OuttakeCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    /**
     * Run the motor backwards
     */
    public void initialize() {
        m_intakeSubsystem.setSpeed(-IntakeConstants.kIntakeSpeed);
    }
}