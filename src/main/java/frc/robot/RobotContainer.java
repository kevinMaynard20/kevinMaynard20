package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.Constants.ControllerConstants.DPad;
import frc.robot.Constants.FieldLocation;
import frc.robot.Constants.LoggingConstants;
import frc.robot.commands.armcommands.DriveArmCommand;
import frc.robot.commands.armcommands.ExtendArmCommand;
import frc.robot.commands.armcommands.RetractArmCommand;
import frc.robot.commands.carouselcommands.FasterCarouselCommand;
import frc.robot.commands.carouselcommands.ReverseCarouselCommand;
import frc.robot.commands.carouselcommands.RunCarouselCommand;
import frc.robot.commands.climbercommands.DriveScissorsCommand;
import frc.robot.commands.drivecommands.ArcadeDriveCommand;
import frc.robot.commands.drivecommands.LimelightCompleteCommand;
import frc.robot.commands.drivecommands.LimelightTurnCommand;
import frc.robot.commands.drivecommands.PixyTargetCommand;
import frc.robot.commands.drivecommands.TrajectoryFollow;
import frc.robot.commands.feedercommands.AutoFeederCommand;
import frc.robot.commands.intakecommands.IntakeCommand;
import frc.robot.commands.intakecommands.OuttakeCommand;
import frc.robot.commands.shootcommands.HoodPositionCommand;
import frc.robot.commands.shootcommands.LimelightShootSetupCommand;
import frc.robot.commands.shootcommands.ShootSetupCommand;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArmSubsystem;
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
	private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
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
	private final ShuffleboardLogging[] m_subsystems = { m_arduinoSubsystem, m_armSubsystem, m_carouselSubsystem,
			m_climberSubsystem, m_driveSubsystem, m_feederSubsystem, m_flywheelSubsystem, m_hoodSubsystem,
			m_intakeSubsystem, m_limelightSubsystem };

	public RobotContainer() {
		// configureButtonBindings();
		configureTestingBindings();
		configureShuffleboard();
		// Generate all trajectories at startup to prevent loop overrun
		generateTrajectoryCommands();
	}

	private void configureButtonBindings() {
		// Driver
		// Drive
		m_driveSubsystem.setDefaultCommand(
				new ArcadeDriveCommand(m_driveSubsystem, () -> -m_driverController.getRawAxis(Axis.kLeftY),
						() -> (m_driverController.getRawAxis(Axis.kLeftTrigger) + 1) / 2,
						() -> (m_driverController.getRawAxis(Axis.kRightTrigger) + 1) / 2));
		// Climber
		m_climberSubsystem.setDefaultCommand(
				new DriveScissorsCommand(m_climberSubsystem, () -> -m_driverController.getRawAxis(Axis.kRightY)));
		// Automatic travel to set distance and shoot setup
		new JoystickButton(m_driverController, Button.kLeftBumper).whenHeld(new ParallelCommandGroup(
				new LimelightCompleteCommand(m_limelightSubsystem, m_driveSubsystem,
						() -> FieldLocation.fromDistance(m_limelightSubsystem.getAverageDistance())),
				new RunCarouselCommand(m_carouselSubsystem), new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem,
						() -> FieldLocation.fromDistance(m_limelightSubsystem.getAverageDistance()))));
		// Automatic turn towards target and shoot setup not using setpoints
		new JoystickButton(m_driverController, Button.kTriangle)
				.whenHeld(new ParallelCommandGroup(new LimelightTurnCommand(m_limelightSubsystem, m_driveSubsystem, 0),
						new RunCarouselCommand(m_carouselSubsystem),
						new LimelightShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, m_limelightSubsystem)));
		// Shoot when other systems are ready
		new JoystickButton(m_driverController, Button.kRightBumper).whenHeld(new ParallelCommandGroup(
				new AutoFeederCommand(m_feederSubsystem, () -> m_carouselSubsystem.getPosition(),
						() -> m_flywheelSubsystem.atSetpoint(), () -> m_hoodSubsystem.atSetpoint())));
		// Pixy ball follow
		new JoystickButton(m_driverController, Button.kX)
				.whenHeld(new ParallelCommandGroup(new PixyTargetCommand(m_driveSubsystem, m_arduinoSubsystem),
						new FasterCarouselCommand(m_carouselSubsystem), new IntakeCommand(m_intakeSubsystem)));

		// Operator
		// Intake
		new JoystickButton(m_operatorController, Button.kX).whenHeld(new ParallelCommandGroup(
				new IntakeCommand(m_intakeSubsystem), new FasterCarouselCommand(m_carouselSubsystem)));
		new JoystickButton(m_operatorController, Button.kCircle).whenHeld(new OuttakeCommand(m_intakeSubsystem));
		// Arm
		new JoystickButton(m_operatorController, Button.kLeftBumper).whenPressed(new RetractArmCommand(m_armSubsystem));
		new JoystickButton(m_operatorController, Button.kRightBumper).whenPressed(new ExtendArmCommand(m_armSubsystem));
		m_armSubsystem.setDefaultCommand(
				new DriveArmCommand(m_armSubsystem, () -> (m_operatorController.getRawAxis(Axis.kRightTrigger) + 1) / 2
						- (m_operatorController.getRawAxis(Axis.kLeftTrigger) + 1) / 2));
		// Carousel jostle
		new JoystickButton(m_operatorController, Button.kTriangle)
				.whenHeld(new ReverseCarouselCommand(m_carouselSubsystem));
		// Hood and flywheel override
		new JoystickButton(m_operatorController, Button.kSquare)
				.whenPressed(new HoodPositionCommand(m_hoodSubsystem, 0));
		new JoystickButton(m_operatorController, DPad.kDown)
				.whenPressed(new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () -> FieldLocation.WALL));
		new JoystickButton(m_operatorController, DPad.kLeft)
				.whenPressed(new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () -> FieldLocation.INITLINE));
		new JoystickButton(m_operatorController, DPad.kUp).whenPressed(
				new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () -> FieldLocation.CLOSETRENCH));
		new JoystickButton(m_operatorController, DPad.kRight).whenPressed(
				new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () -> FieldLocation.FARTRENCH));
		// Zero hood encoder
		new JoystickButton(m_operatorController, Button.kShare).whenPressed(() -> m_hoodSubsystem.resetEncoder());
		// Zero carousel encoder
		new JoystickButton(m_operatorController, Button.kOptions).whenPressed(() -> m_carouselSubsystem.resetEncoder());
	}

	private void configureTestingBindings() {
		// Serves to switch between testing and actual controls more quickly than
		// commenting everything out
		m_driveSubsystem.setDefaultCommand(
				new ArcadeDriveCommand(m_driveSubsystem, () -> -m_driverController.getRawAxis(Axis.kLeftY),
						() -> (m_driverController.getRawAxis(Axis.kLeftTrigger) + 1) / 2,
						() -> (m_driverController.getRawAxis(Axis.kRightTrigger) + 1) / 2));
		// Zero hood encoder
		new JoystickButton(m_driverController, Button.kShare).whenPressed(() -> m_hoodSubsystem.resetEncoder());
		// Zero carousel encoder
		new JoystickButton(m_driverController, Button.kOptions).whenPressed(() -> m_carouselSubsystem.resetEncoder());

	}

	public void configureShuffleboard() {
		for (int i = 0; i < m_subsystems.length; i++) {
			if (LoggingConstants.kSubsystems[i]) {
				m_subsystems[i].configureShuffleboard();
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