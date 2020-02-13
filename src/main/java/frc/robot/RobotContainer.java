package frc.robot;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.geometry.*;
import frc.robot.Constants.*;
import frc.robot.Constants.ControllerConstants.*;
import frc.robot.commands.carouselcommands.*;
import frc.robot.commands.drivecommands.*;
import frc.robot.commands.feedercommands.*;
import frc.robot.subsystems.*;

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
	// auto selector
	private final SendableChooser<RamseteCommand> m_autoChooser = new SendableChooser<>();

	public RobotContainer() {
		configureButtonBindings();

		// Generate all trajectories at startup to prevent loop overrun
		generateTrajectoryCommands();

		m_driveSubsystem.setDefaultCommand(
				new VelocityDriveCommand(m_driveSubsystem, () -> -m_driverController.getRawAxis(Axis.kLeftY),
						() -> (m_driverController.getRawAxis(Axis.kLeftTrigger) + 1) / 2,
						() -> (m_driverController.getRawAxis(Axis.kRightTrigger) + 1) / 2));
	}

	private void configureButtonBindings() {
		new JoystickButton(m_driverController, ControllerConstants.Button.kTriangle)
				.whenHeld(new PixyTargetCommand(m_driveSubsystem, m_arduinoSubsystem));

		new JoystickButton(m_driverController, ControllerConstants.Button.kCircle)
				.whenHeld(new CarouselCommand(m_carouselSubsystem));

		new JoystickButton(m_driverController, ControllerConstants.Button.kX)
				.whenHeld(new FeederCommand(m_feederSubsystem, () -> m_carouselSubsystem.getPosition()));

		new POVButton(m_driverController, ControllerConstants.DPad.kDown)
				.whenPressed(() -> m_flywheelSubsystem.setSetpoint(1000), m_flywheelSubsystem);
		new POVButton(m_driverController, ControllerConstants.DPad.kLeft)
				.whenPressed(() -> m_flywheelSubsystem.setSetpoint(1500), m_flywheelSubsystem);
		new POVButton(m_driverController, ControllerConstants.DPad.kUp)
				.whenPressed(() -> m_flywheelSubsystem.setSetpoint(2000), m_flywheelSubsystem);
		new POVButton(m_driverController, ControllerConstants.DPad.kRight)
				.whenPressed(() -> m_flywheelSubsystem.setSetpoint(2500), m_flywheelSubsystem);

		new JoystickButton(m_driverController, ControllerConstants.Button.kLeftBumper)
				.whenPressed(() -> m_flywheelSubsystem.setSetpoint(0));

		new JoystickButton(m_driverController, ControllerConstants.Button.kSquare)
				.whenPressed(() -> CommandScheduler.getInstance().cancelAll());

		new JoystickButton(m_driverController, ControllerConstants.Button.kOptions)
				.whenPressed(() -> m_carouselSubsystem.setPosition(0));
	}

	public Command getAutonomousCommand() {
		return m_autoChooser.getSelected();
	}

	public void generateTrajectoryCommands() {
		ArrayList<String> trajectoryCommandNames = new ArrayList<>();
		ArrayList<Trajectory> trajectories = new ArrayList<>();

		trajectoryCommandNames.add("Straight Line");
		trajectories.add(TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
				List.of(new Translation2d(1, 0), new Translation2d(2, 0)), new Pose2d(3, 0, new Rotation2d(0)),
				DriveConstants.kTrajectoryConfig));

		trajectoryCommandNames.add("Zig Zag");
		trajectories.add(TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
				List.of(new Translation2d(1, -.2), new Translation2d(2, .2), new Translation2d(3, -.2),
						new Translation2d(4, .2)),
				new Pose2d(5, 0, new Rotation2d(0)), DriveConstants.kTrajectoryConfig));

		for (int i = 0; i < trajectories.size(); i++) {
			m_autoChooser.addOption(trajectoryCommandNames.get(i),
					new RamseteCommand(trajectories.get(i), m_driveSubsystem::getPose, new RamseteController(),
							DriveConstants.kFeedForward, DriveConstants.kDriveKinematics,
							m_driveSubsystem::getWheelSpeeds, new PIDController(DriveConstants.kPDriveVel, 0, 0),
							new PIDController(DriveConstants.kPDriveVel, 0, 0), m_driveSubsystem::tankDriveVolts,
							m_driveSubsystem));
		}
	}
}