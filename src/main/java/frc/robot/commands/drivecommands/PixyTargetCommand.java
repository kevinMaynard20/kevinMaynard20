package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class PixyTargetCommand extends CommandBase {

    private final DriveSubsystem m_driveSubsystem;
    private final ArduinoSubsystem m_arduinoSubsystem;

    /**
     * Initializes a new instance of the {@link TargetCommand} class.
     * 
     * @param driveSubsystem   {@link DriveSubsystem} to be used.
     * @param arduinoSubsystem {@link ArduinoSubsystem} to be used.
     */
    public PixyTargetCommand(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_arduinoSubsystem = arduinoSubsystem;
        addRequirements(driveSubsystem, arduinoSubsystem);
    }

    public void execute() {
        // read and write Arduino data
        m_arduinoSubsystem.update();
        // if target is in the camera's view
        if (m_arduinoSubsystem.getTargetInView())
            // drive based on Arduino data
            // TODO: use Math.max() to make min speed of 0.4
            m_driveSubsystem.arcadeDrive(-m_arduinoSubsystem.getDriveSpeed(), m_arduinoSubsystem.getTurnSpeed(),
                    -m_arduinoSubsystem.getTurnSpeed());
        else
            m_driveSubsystem.tankDrive(0, 0);
    }
}