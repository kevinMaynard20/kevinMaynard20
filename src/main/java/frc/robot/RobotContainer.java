package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.Constants.ControllerConstants.DPad;
import frc.robot.Constants.FieldLocation;
import frc.robot.commands.carouselcommands.CarouselCommand;
import frc.robot.commands.climbercommands.DriveScissorsCommand;
import frc.robot.commands.climbercommands.LowerScissorsCommand;
import frc.robot.commands.climbercommands.RaiseScissorsCommand;
import frc.robot.commands.drivecommands.ArcadeDriveCommand;
import frc.robot.commands.drivecommands.LimelightCompleteCommand;
import frc.robot.commands.drivecommands.TrajectoryFollow;
import frc.robot.commands.feedercommands.FeederCommand;
import frc.robot.commands.intakecommands.IntakeCommand;
import frc.robot.commands.intakecommands.OuttakeCommand;
import frc.robot.commands.intakecommands.StopIntakeCommand;
import frc.robot.commands.shootcommands.HoodPositionCommand;
import frc.robot.commands.shootcommands.LimelightShootCommand;
import frc.robot.commands.shootcommands.ShootCommand;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.CarouselSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class RobotContainer {
	// subsystems
	private final ArduinoSubsystem m_arduinoSubsystem = new ArduinoSubsystem();
	private final CarouselSubsystem m_carouselSubsystem = new CarouselSubsystem();
	private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final FeederSubsystem m_feederSubsystem = new FeederSubsystem();
	private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
	private final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();
	private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
	private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
	// controllers
	private final Joystick m_driverController = new Joystick(ControllerConstants.kDriverControllerPort);
	private final Joystick m_operatorController = new Joystick(ControllerConstants.kOperatorControllerPort);
	// auto selector
	private final SendableChooser<RamseteCommand> m_autoChooser = new SendableChooser<>();
	// shuffleboard logging parallel arrays
	private final ShuffleboardLogging[] m_subsystems = { m_arduinoSubsystem, m_carouselSubsystem, m_climberSubsystem,
			m_driveSubsystem, m_feederSubsystem, m_flywheelSubsystem, m_hoodSubsystem, m_intakeSubsystem,
			m_limelightSubsystem };
	private final SimpleWidget[] m_viewToggleWidgets = new SimpleWidget[m_subsystems.length];

	public RobotContainer() {
		configureButtonBindings();
		configureShuffleboard();

		// Generate all trajectories at startup to prevent loop overrun
		generateTrajectoryCommands();

		m_driveSubsystem.setDefaultCommand(
				new ArcadeDriveCommand(m_driveSubsystem, () -> -m_driverController.getRawAxis(Axis.kLeftY),
						() -> (m_driverController.getRawAxis(Axis.kLeftTrigger) + 1) / 2,
						() -> (m_driverController.getRawAxis(Axis.kRightTrigger) + 1) / 2));
	}

	private void configureButtonBindings() {
		// Driver
		// Automatic everything
		new JoystickButton(m_driverController, Button.kRightBumper).whenHeld(new ParallelCommandGroup(
				new LimelightCompleteCommand(m_limelightSubsystem, m_driveSubsystem, () -> 0.0, () -> 0.0),
				new CarouselCommand(m_carouselSubsystem),
				new FeederCommand(m_feederSubsystem, () -> m_carouselSubsystem.getPosition()),
				new LimelightShootCommand(m_flywheelSubsystem, m_hoodSubsystem, m_limelightSubsystem)));
		// Climber
		new JoystickButton(m_driverController, DPad.kUp).whenPressed(new RaiseScissorsCommand(m_climberSubsystem));
		new JoystickButton(m_driverController, DPad.kDown).whenPressed(new LowerScissorsCommand(m_climberSubsystem));
		// Climber override
		new JoystickButton(m_driverController, DPad.kLeft).whenHeld(
				new DriveScissorsCommand(m_climberSubsystem, () -> m_driverController.getRawAxis(Axis.kRightY)));

		// Operator
		new JoystickButton(m_operatorController, Button.kX).whenPressed(new IntakeCommand(m_intakeSubsystem));
		new JoystickButton(m_operatorController, Button.kSquare).whenPressed(new OuttakeCommand(m_intakeSubsystem));
		new JoystickButton(m_operatorController, Button.kCircle).whenPressed(new StopIntakeCommand(m_intakeSubsystem));
		new JoystickButton(m_operatorController, DPad.kDown)
				.whenPressed(new ShootCommand(m_flywheelSubsystem, m_hoodSubsystem, FieldLocation.WALL));
		new JoystickButton(m_operatorController, DPad.kLeft)
				.whenPressed(new ShootCommand(m_flywheelSubsystem, m_hoodSubsystem, FieldLocation.INITLINE));
		new JoystickButton(m_operatorController, DPad.kUp)
				.whenPressed(new ShootCommand(m_flywheelSubsystem, m_hoodSubsystem, FieldLocation.CLOSETRENCH));
		new JoystickButton(m_operatorController, DPad.kRight)
				.whenPressed(new ShootCommand(m_flywheelSubsystem, m_hoodSubsystem, FieldLocation.FARTRENCH));
		new JoystickButton(m_operatorController, Button.kPS).whenPressed(new HoodPositionCommand(m_hoodSubsystem, 0));
	}

	private void configureShuffleboard() {
		// Put switches on shuffleboard to enable more intracite diagnostic views
		ShuffleboardTab viewToggleTab = Shuffleboard.getTab("Toggle Views");
		m_viewToggleWidgets[0] = viewToggleTab.add("Arduino", 0).withSize(1, 1).withPosition(0, 0)
				.withWidget(BuiltInWidgets.kToggleSwitch);
		m_viewToggleWidgets[1] = viewToggleTab.add("Carousel", 0).withSize(1, 1).withPosition(1, 0)
				.withWidget(BuiltInWidgets.kToggleSwitch);
		m_viewToggleWidgets[2] = viewToggleTab.add("Climber", 0).withSize(1, 1).withPosition(2, 0)
				.withWidget(BuiltInWidgets.kToggleSwitch);
		m_viewToggleWidgets[3] = viewToggleTab.add("Drive", 0).withSize(1, 1).withPosition(3, 0)
				.withWidget(BuiltInWidgets.kToggleSwitch);
		m_viewToggleWidgets[4] = viewToggleTab.add("Feeder", 0).withSize(1, 1).withPosition(4, 0)
				.withWidget(BuiltInWidgets.kToggleSwitch);
		m_viewToggleWidgets[5] = viewToggleTab.add("Flywheel", 0).withSize(1, 1).withPosition(5, 0)
				.withWidget(BuiltInWidgets.kToggleSwitch);
		m_viewToggleWidgets[6] = viewToggleTab.add("Hood", 0).withSize(1, 1).withPosition(6, 0)
				.withWidget(BuiltInWidgets.kToggleSwitch);
		m_viewToggleWidgets[7] = viewToggleTab.add("Intake", 0).withSize(1, 1).withPosition(7, 0)
				.withWidget(BuiltInWidgets.kToggleSwitch);
		m_viewToggleWidgets[8] = viewToggleTab.add("Limelight", 0).withSize(1, 1).withPosition(8, 0)
				.withWidget(BuiltInWidgets.kToggleSwitch);
	}

	public void updateShuffleboard() {
		// Iterate through all switches and update shuffleboard for enabled subsystems
		for (int i = 0; i < m_subsystems.length; i++) {
			if (m_viewToggleWidgets[i].getEntry().getBoolean(false)) {
				m_subsystems[i].updateShuffleboard(Shuffleboard.getTab(m_viewToggleWidgets[i].getTitle()));
			}
		}
	}

	public Command getAutonomousCommand() {
		return m_autoChooser.getSelected();
	}

	private void generateTrajectoryCommands() {
		// Generate trajectories that will be strung into auto commands before putting
		// them onto shuffleboard to be selected
		String trajectoryJSON = "paths/Path1.wpilib.json";
		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
			Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			m_autoChooser.addOption("Trajectory", new TrajectoryFollow(m_driveSubsystem, trajectory));
		} catch (IOException ex) {
			Shuffleboard.getTab("Errors").add("Trajectory Error", ex.getStackTrace().toString()).withSize(4, 4)
					.withPosition(0, 0).withWidget(BuiltInWidgets.kTextView);
		}
		SmartDashboard.putData(m_autoChooser);
	}
}