package frc.robot.commands.armcommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.ArmSubsystem;

public class DriveArmCommand extends CommandBase {

    private final ArmSubsystem m_armSubsystem;
    private final Supplier<Double> m_speed;

    /**
     * Drive the hood using percent output
     * 
     * @param armSubsystem The hood subsystem to be used
     * @param speed        Percent output supplier
     */
    public DriveArmCommand(ArmSubsystem armSubsystem, Supplier<Double> speed) {
        m_armSubsystem = armSubsystem;
        m_speed = speed;
        addRequirements(m_armSubsystem);
    }

    /**
     * Update the motor output
     */
    public void execute() {
        double speed = Math.abs(m_speed.get()) > ControllerConstants.kTriggerDeadzone ? m_speed.get() : 0;
        m_armSubsystem.setPercentOutput(speed);
    }

    /**
     * Set the setposition to the current position when the command ends
     */
    public void end(boolean interrupted) {
        m_armSubsystem.setPercentOutput(0);
    }
}