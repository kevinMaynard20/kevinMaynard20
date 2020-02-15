package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.CarouselCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.TargetCommand;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.CarouselSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;

public class RobotContainer {
	// subsystems
	private final ArduinoSubsystem m_arduinoSubsystem = new ArduinoSubsystem();
	private final CarouselSubsystem m_carouselSubsystem = new CarouselSubsystem();
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final FeederSubsystem m_feederSubsystem = new FeederSubsystem();
	private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
	// controllers
	private final Joystick m_driverController = new Joystick(ControllerConstants.kDriverControllerPort);
	private final Joystick m_operatorController = new Joystick(ControllerConstants.kOperatorControllerPort);

	public RobotContainer() {
		configureButtonBindings();

		m_driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(m_driveSubsystem,
				() -> -m_driverController.getRawAxis(ControllerConstants.Axis.kLeftY),
				() -> (m_driverController.getRawAxis(ControllerConstants.Axis.kLeftTrigger) + 1) / 2,
				() -> (m_driverController.getRawAxis(ControllerConstants.Axis.kRightTrigger) + 1) / 2));
	}

	private void configureButtonBindings() {
		new JoystickButton(m_driverController, ControllerConstants.Button.kTriangle)
				.whenHeld(new TargetCommand(m_driveSubsystem, m_arduinoSubsystem));

		new JoystickButton(m_driverController, ControllerConstants.Button.kCircle)
				.toggleWhenPressed(new CarouselCommand(m_carouselSubsystem));

		new JoystickButton(m_driverController, ControllerConstants.Button.kX)
				.toggleWhenPressed(new FeederCommand(m_feederSubsystem, () -> m_carouselSubsystem.getPosition()));

		new POVButton(m_driverController, ControllerConstants.DPad.kDown)
				.whenPressed(() -> m_flywheelSubsystem.setSetpoint(5000), m_flywheelSubsystem);
		new POVButton(m_driverController, ControllerConstants.DPad.kLeft)
				.whenPressed(() -> m_flywheelSubsystem.setSetpoint(6000), m_flywheelSubsystem);
		new POVButton(m_driverController, ControllerConstants.DPad.kUp)
				.whenPressed(() -> m_flywheelSubsystem.setSetpoint(7000), m_flywheelSubsystem);
		new POVButton(m_driverController, ControllerConstants.DPad.kRight)
				.whenPressed(() -> m_flywheelSubsystem.setSetpoint(8000), m_flywheelSubsystem);

		new JoystickButton(m_driverController, ControllerConstants.Button.kLeftBumper)
				.whenPressed(() -> m_flywheelSubsystem.setSetpoint(0));

		new JoystickButton(m_driverController, ControllerConstants.Button.kSquare)
				.whenPressed(() -> CommandScheduler.getInstance().cancelAll());

		new JoystickButton(m_driverController, ControllerConstants.Button.kOptions)
				.whenPressed(() -> m_carouselSubsystem.setPosition(0));
	}
}