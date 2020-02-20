package frc.robot.commands.shootcommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.FlywheelSubsystem;

public class DriveFlywheelCommand extends CommandBase {

    private final FlywheelSubsystem m_flywheelSubsystem;
    private final Supplier<Double> m_speed;

    /**
     * 
     * @param flywheelSubsystem
     * @param hoodSubsystem
     * @param limelightSubsystem
     */
    public DriveFlywheelCommand(FlywheelSubsystem flywheelSubsystem, Supplier<Double> speed) {
        m_flywheelSubsystem = flywheelSubsystem;
        m_speed = speed;
        addRequirements(flywheelSubsystem);
    }

    /**
     * Update the motor setpoint
     */
    public void execute() {
        m_flywheelSubsystem.setSetpoint(FlywheelConstants.kMaxRPM * m_speed.get());
    }

}
