package frc.robot;

import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Hashtable;

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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CarouselConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.Constants.ControllerConstants.DPad;
import frc.robot.Constants.FieldLocation;
import frc.robot.Constants.LoggingConstants;
import frc.robot.commands.armcommands.BounceArmCommand;
import frc.robot.commands.armcommands.DriveArmCommand;
import frc.robot.commands.armcommands.ExtendArmCommand;
import frc.robot.commands.armcommands.RetractArmCommand;
import frc.robot.commands.carouselcommands.AutoSpeedCarouselCommand;
import frc.robot.commands.carouselcommands.RunCarouselCommand;
import frc.robot.commands.carouselcommands.ToOpenSpaceCommand;
import frc.robot.commands.climbercommands.DriveScissorsCommand;
import frc.robot.commands.drivecommands.ArcadeDriveCommand;
import frc.robot.commands.drivecommands.LimelightTurnCommand;
import frc.robot.commands.drivecommands.PixyTargetCommand;
import frc.robot.commands.drivecommands.TrajectoryFollow;
import frc.robot.commands.feedercommands.AutoFeederCommand;
import frc.robot.commands.feedercommands.FeederCommand;
import frc.robot.commands.feedercommands.ReverseFeederCommand;
import frc.robot.commands.intakecommands.IntakeCommand;
import frc.robot.commands.intakecommands.OuttakeCommand;
import frc.robot.commands.shootcommands.DriveHoodCommand;
import frc.robot.commands.shootcommands.HoodPositionCommand;
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
		configureButtonBindings();
		// configureTestingBindings();
		configureShuffleboard();
		// Generate all trajectories at startup to prevent loop overrun
		// generateTrajectoryCommands();
	}

	private void configureButtonBindings() {
		// Driver
		// Drive arcade
		// m_driveSubsystem.setDefaultCommand(
		// new CurvatureDriveCommand(m_driveSubsystem, () ->
		// -m_driverController.getRawAxis(Axis.kLeftY),
		// () -> (m_driverController.getRawAxis(Axis.kLeftTrigger) + 1) / 2,
		// () -> (m_driverController.getRawAxis(Axis.kRightTrigger) + 1) / 2));
		m_driveSubsystem.setDefaultCommand(
				new ArcadeDriveCommand(m_driveSubsystem, () -> -m_driverController.getRawAxis(Axis.kLeftY),
						() -> (m_driverController.getRawAxis(Axis.kLeftTrigger) + 1) / 2,
						() -> (m_driverController.getRawAxis(Axis.kRightTrigger) + 1) / 2));
		// Climber
		m_climberSubsystem.setDefaultCommand(
				new DriveScissorsCommand(m_climberSubsystem, () -> -m_driverController.getRawAxis(Axis.kRightY)));
		// Carousel default
		m_carouselSubsystem.setDefaultCommand(new ToOpenSpaceCommand(m_carouselSubsystem));

		// Pixy ball follow
		new JoystickButton(m_driverController, Button.kX).whenHeld(new ParallelCommandGroup(
				new PixyTargetCommand(m_driveSubsystem, m_arduinoSubsystem,
						() -> -m_driverController.getRawAxis(Axis.kLeftY)),
				new RunCarouselCommand(m_carouselSubsystem,
						CarouselConstants.kIntakeVelocity * CarouselConstants.kRatio),
				new IntakeCommand(m_intakeSubsystem), new BounceArmCommand(m_armSubsystem)));
		// Turn to target
		new POVButton(m_driverController, DPad.kUp)
				.whenHeld(new LimelightTurnCommand(m_limelightSubsystem, m_driveSubsystem, 0));
		// Feeder
		new JoystickButton(m_driverController, Button.kLeftBumper).whenHeld(new FeederCommand(m_feederSubsystem));
		// Run carousel fast
		new JoystickButton(m_driverController, Button.kRightBumper)
				.whenHeld(new AutoSpeedCarouselCommand(m_carouselSubsystem, m_flywheelSubsystem::getSetpoint));
		// Run carousel default speed
		new POVButton(m_driverController, DPad.kDown).toggleWhenPressed(
				new RunCarouselCommand(m_carouselSubsystem, CarouselConstants.kVelocity * CarouselConstants.kRatio));
		new POVButton(m_driverController, DPad.kRight).whenHeld(new ReverseFeederCommand(m_feederSubsystem));

		// Operator
		// Intake
		new JoystickButton(m_operatorController, Button.kX)
				.whenHeld(new ParallelCommandGroup(new IntakeCommand(m_intakeSubsystem),
						new RunCarouselCommand(m_carouselSubsystem,
								CarouselConstants.kIntakeVelocity * CarouselConstants.kRatio)))
				.whenHeld(new BounceArmCommand(m_armSubsystem));
		new JoystickButton(m_operatorController, Button.kCircle).whenHeld(new OuttakeCommand(m_intakeSubsystem));
		// Arm
		new JoystickButton(m_operatorController, Button.kLeftBumper).whenPressed(new RetractArmCommand(m_armSubsystem));
		new JoystickButton(m_operatorController, Button.kRightBumper).whenPressed(new ExtendArmCommand(m_armSubsystem));
		new Trigger(() -> Math.abs(
				(m_operatorController.getRawAxis(Axis.kLeftTrigger) + 1) / 2) > ControllerConstants.kTriggerDeadzone
				|| Math.abs(
						(m_operatorController.getRawAxis(Axis.kRightTrigger) + 1) / 2) > ControllerConstants.kDeadzone)
								.whenActive(new DriveArmCommand(m_armSubsystem,
										() -> (m_operatorController.getRawAxis(Axis.kLeftTrigger) + 1) / 2,
										() -> (m_operatorController.getRawAxis(Axis.kRightTrigger) + 1) / 2));
		m_armSubsystem.setDefaultCommand(
				new DriveArmCommand(m_armSubsystem, () -> (m_operatorController.getRawAxis(Axis.kLeftTrigger) + 1) / 2,
						() -> (m_operatorController.getRawAxis(Axis.kRightTrigger) + 1) / 2));
		// Carousel jostle
		new JoystickButton(m_operatorController, Button.kTriangle).whenHeld(new RunCarouselCommand(m_carouselSubsystem,
				CarouselConstants.kJostleVelocity * CarouselConstants.kRatio));
		// Hood and flywheel override
		new POVButton(m_operatorController, DPad.kDown).whenHeld(new ParallelCommandGroup(
				new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () -> FieldLocation.WALL),
				new AutoFeederCommand(m_feederSubsystem, m_carouselSubsystem::atOpenSpace,
						m_flywheelSubsystem::atSetpoint)));
		new POVButton(m_operatorController, DPad.kLeft).whenHeld(new ParallelCommandGroup(
				new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () -> FieldLocation.INITLINE),
				new AutoFeederCommand(m_feederSubsystem, m_carouselSubsystem::atOpenSpace,
						m_flywheelSubsystem::atSetpoint)));
		new POVButton(m_operatorController, DPad.kUp).whenHeld(new ParallelCommandGroup(
				new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () -> FieldLocation.CLOSETRENCH),
				new AutoFeederCommand(m_feederSubsystem, m_carouselSubsystem::atOpenSpace,
						m_flywheelSubsystem::atSetpoint)));
		new POVButton(m_operatorController, DPad.kRight).whenHeld(new ParallelCommandGroup(
				new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () -> FieldLocation.FARTRENCH),
				new AutoFeederCommand(m_feederSubsystem, m_carouselSubsystem::atOpenSpace,
						m_flywheelSubsystem::atSetpoint)));
		// Zero hood encoder
		new JoystickButton(m_operatorController, Button.kShare).whenPressed(() -> m_hoodSubsystem.resetEncoder());
		// Zero carousel encoder
		new JoystickButton(m_operatorController, Button.kOptions).whenPressed(() -> m_carouselSubsystem.resetEncoder());
		// Zero arm encoder
		new JoystickButton(m_operatorController, Button.kTrackpad).whenPressed(() -> m_armSubsystem.resetEncoder());
	}

	private void configureTestingBindings() {
		// driving
		m_driveSubsystem.setDefaultCommand(
				new ArcadeDriveCommand(m_driveSubsystem, () -> -m_driverController.getRawAxis(Axis.kLeftY),
						() -> (m_driverController.getRawAxis(Axis.kLeftTrigger) + 1) / 2,
						() -> (m_driverController.getRawAxis(Axis.kRightTrigger) + 1) / 2));
		// // flywheel
		// new POVButton(m_operatorController, DPad.kUp).whenPressed(() ->
		// m_flywheelSubsystem.incrementSpeed(),
		// m_flywheelSubsystem);
		// new POVButton(m_operatorController, DPad.kDown).whenPressed(() ->
		// m_flywheelSubsystem.decrementSpeed(),
		// m_flywheelSubsystem);
		// new POVButton(m_operatorController,
		// DPad.kLeft).whenPressed(()->m_flywheelSubsystem.setVelocity(5000));
		// new POVButton(m_operatorController,
		// DPad.kRight).whenPressed(()->m_flywheelSubsystem.setVelocity(6000));
		// new JoystickButton(m_operatorController, Button.kLeftBumper)
		// .whenPressed(() -> m_flywheelSubsystem.setVelocity(0));
		// Hood and flywheel override
		new POVButton(m_operatorController, DPad.kDown).whenHeld(new ParallelCommandGroup(
				new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () -> FieldLocation.WALL),
				new AutoFeederCommand(m_feederSubsystem, m_carouselSubsystem::atOpenSpace,
						m_flywheelSubsystem::atSetpoint)));
		new POVButton(m_operatorController, DPad.kLeft).whenHeld(new ParallelCommandGroup(
				new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () -> FieldLocation.TWOFEET),
				new AutoFeederCommand(m_feederSubsystem, m_carouselSubsystem::atOpenSpace,
						m_flywheelSubsystem::atSetpoint)));
		new POVButton(m_operatorController, DPad.kUp).whenHeld(new ParallelCommandGroup(
				new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () -> FieldLocation.INITLINE),
				new AutoFeederCommand(m_feederSubsystem, m_carouselSubsystem::atOpenSpace,
						m_flywheelSubsystem::atSetpoint)));
		new POVButton(m_operatorController, DPad.kRight).whenHeld(new ParallelCommandGroup(
				new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () -> FieldLocation.CLOSETRENCH),
				new AutoFeederCommand(m_feederSubsystem, m_carouselSubsystem::atOpenSpace,
						m_flywheelSubsystem::atSetpoint)));
		// // hood
		// m_hoodSubsystem.setDefaultCommand(
		// new DriveHoodCommand(m_hoodSubsystem, () ->
		// m_operatorController.getRawAxis(Axis.kRightY) * 0.1));
		// new JoystickButton(m_operatorController, Button.kOptions).whenPressed(() ->
		// m_hoodSubsystem.resetEncoder());
		// new JoystickButton(m_operatorController, Button.kRightBumper).whenPressed(new
		// HoodPositionCommand(m_hoodSubsystem, 0));
		// arm
		m_armSubsystem.setDefaultCommand(
				new DriveArmCommand(m_armSubsystem, () -> (m_operatorController.getRawAxis(Axis.kLeftTrigger) + 1) / 2,
						() -> (m_operatorController.getRawAxis(Axis.kRightTrigger) + 1) / 2));
		// intake
		new JoystickButton(m_operatorController, Button.kX).whenHeld(new IntakeCommand(m_intakeSubsystem));
		// carousel
		new JoystickButton(m_operatorController, Button.kCircle).toggleWhenPressed(
				new RunCarouselCommand(m_carouselSubsystem, CarouselConstants.kVelocity * CarouselConstants.kRatio));
		// feeder
		new JoystickButton(m_operatorController, Button.kTriangle).whenHeld(new FeederCommand(m_feederSubsystem));
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

	/**
	 * Generates all autonomous commands.
	 */
	private void generateAutonomousCommands() {
		Hashtable<String, Trajectory> trajectories = new Hashtable<String, Trajectory>();
		File[] files = new File("paths").listFiles();
		for (File file : files)
			try {
				trajectories.put(file.getName(), TrajectoryUtil
						.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(file.getPath())));
			} catch (IOException e) {
				Shuffleboard.getTab("Errors").add("Trajectory Error", e.getStackTrace().toString()).withSize(4, 4)
						.withPosition(0, 0).withWidget(BuiltInWidgets.kTextView);
			}
		for (String name : trajectories.keySet())
			m_autoChooser.addOption(name, new TrajectoryFollow(m_driveSubsystem, trajectories.get(name)));
	}
}