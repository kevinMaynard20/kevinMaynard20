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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.commands.drivecommands.ArcadeDriveCommand;
import frc.robot.commands.drivecommands.TrajectoryFollow;
import frc.robot.commands.shootcommands.DriveHoodCommand;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.CarouselSubsystem;
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
	private final ShuffleboardLogging[] m_subsystems = { m_arduinoSubsystem, m_carouselSubsystem, m_driveSubsystem,
			m_feederSubsystem, m_flywheelSubsystem, m_hoodSubsystem, m_intakeSubsystem, m_limelightSubsystem };
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

		m_hoodSubsystem.setDefaultCommand(
				new DriveHoodCommand(m_hoodSubsystem, () -> m_driverController.getRawAxis(Axis.kRightY) / 5.0));
	}

	private void configureButtonBindings() {
		// new JoystickButton(m_driverController, ControllerConstants.Button.kTriangle)
		// .whenHeld(new PixyTargetCommand(m_driveSubsystem, m_arduinoSubsystem));

		// new JoystickButton(m_driverController, ControllerConstants.Button.kCircle)
		// .toggleWhenPressed(new CarouselCommand(m_carouselSubsystem));

		// new JoystickButton(m_driverController, ControllerConstants.Button.kX)
		// .toggleWhenPressed(new FeederCommand(m_feederSubsystem, () ->
		// m_carouselSubsystem.getPosition()));

		new POVButton(m_driverController, ControllerConstants.DPad.kDown)
				.whenPressed(() -> m_flywheelSubsystem.setSetpoint(5000));
		new POVButton(m_driverController, ControllerConstants.DPad.kLeft)
				.whenPressed(() -> m_flywheelSubsystem.setSetpoint(5200));
		new POVButton(m_driverController, ControllerConstants.DPad.kUp)
				.whenPressed(() -> m_flywheelSubsystem.setSetpoint(5400));
		new POVButton(m_driverController, ControllerConstants.DPad.kRight)
				.whenPressed(() -> m_flywheelSubsystem.setSetpoint(5600));
		new JoystickButton(m_driverController, ControllerConstants.Button.kLeftBumper)
				.whenPressed(() -> m_flywheelSubsystem.setSetpoint(0));

		// new POVButton(m_driverController, ControllerConstants.DPad.kDown)
		// .whenPressed(() -> m_hoodSubsystem.setSetPoint(-0.0));
		// new POVButton(m_driverController, ControllerConstants.DPad.kLeft)
		// .whenPressed(() -> m_hoodSubsystem.setSetPoint(-10.0));
		// new POVButton(m_driverController, ControllerConstants.DPad.kUp)
		// .whenPressed(() -> m_hoodSubsystem.setSetPoint(-20.0));
		// new POVButton(m_driverController, ControllerConstants.DPad.kRight)
		// .whenPressed(() -> m_hoodSubsystem.setSetPoint(-40.0));
		// new JoystickButton(m_driverController,
		// ControllerConstants.Button.kRightBumper)
		// .whenHeld(new DriveHoodCommand(m_hoodSubsystem, () ->
		// m_driverController.getRawAxis(Axis.kRightY)));

		new JoystickButton(m_driverController, ControllerConstants.Button.kShare)
				.whenPressed(() -> m_hoodSubsystem.resetEncoder());

		new JoystickButton(m_driverController, ControllerConstants.Button.kSquare)
				.whenPressed(() -> CommandScheduler.getInstance().cancelAll());
	}

	private void configureShuffleboard() {
		// Put switches on shuffleboard to enable more intracite diagnostic views
		ShuffleboardTab viewToggleTab = Shuffleboard.getTab("Toggle Views");
		m_viewToggleWidgets[0] = viewToggleTab.add("Arduino", 0).withSize(1, 1).withPosition(0, 0)
				.withWidget(BuiltInWidgets.kToggleSwitch);
		m_viewToggleWidgets[1] = viewToggleTab.add("Carousel", 0).withSize(1, 1).withPosition(1, 0)
				.withWidget(BuiltInWidgets.kToggleSwitch);
		m_viewToggleWidgets[2] = viewToggleTab.add("Drive", 0).withSize(1, 1).withPosition(2, 0)
				.withWidget(BuiltInWidgets.kToggleSwitch);
		m_viewToggleWidgets[3] = viewToggleTab.add("Feeder", 0).withSize(1, 1).withPosition(3, 0)
				.withWidget(BuiltInWidgets.kToggleSwitch);
		m_viewToggleWidgets[4] = viewToggleTab.add("Flywheel", 0).withSize(1, 1).withPosition(1, 1)
				.withWidget(BuiltInWidgets.kToggleSwitch);
		m_viewToggleWidgets[5] = viewToggleTab.add("Hood", 0).withSize(1, 1).withPosition(1, 2)
				.withWidget(BuiltInWidgets.kToggleSwitch);
		m_viewToggleWidgets[6] = viewToggleTab.add("Intake", 0).withSize(1, 1).withPosition(1, 3)
				.withWidget(BuiltInWidgets.kToggleSwitch);
		m_viewToggleWidgets[7] = viewToggleTab.add("Limelight", 0).withSize(1, 1).withPosition(1, 4)
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