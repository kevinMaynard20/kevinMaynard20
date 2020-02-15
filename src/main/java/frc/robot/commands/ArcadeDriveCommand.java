package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends CommandBase {
	private final DriveSubsystem m_driveSubsystem;
	private final Supplier<Double> m_straightSpeed;
	private final Supplier<Double> m_leftSpeed;
	private final Supplier<Double> m_rightSpeed;

	/**
	 * Initializes a new instance of the {@link DriveCommand} class.
	 * 
	 * @param driveSubsystem {@link DriveSubsystem} to be used.
	 * @param straightSpeed  Supplier of driving speed.
	 * @param leftSpeed      Supplier of left turn speed.
	 * @param rightSpeed     Supplier of right turn speed.
	 */
	public ArcadeDriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> straightSpeed, Supplier<Double> leftSpeed,
			Supplier<Double> rightSpeed) {
		m_driveSubsystem = driveSubsystem;
		m_straightSpeed = straightSpeed;
		m_leftSpeed = leftSpeed;
		m_rightSpeed = rightSpeed;

		addRequirements(m_driveSubsystem);
	}

	@Override
    public void execute() {
        double straight = Math.abs(m_straightSpeed.get()) >= 0.1 ? m_straightSpeed.get() : 0.0;
		double left = Math.abs(m_leftSpeed.get()) >= 0.1 ? m_leftSpeed.get() : 0.0;
		double right = Math.abs(m_rightSpeed.get()) >= 0.1 ? m_rightSpeed.get() : 0.0;
        m_driveSubsystem.arcadeDrive(straight, left, right);
    }
}