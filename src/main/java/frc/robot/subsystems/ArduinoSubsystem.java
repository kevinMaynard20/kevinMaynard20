package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArduinoConstants;
import frc.robot.ShuffleboardLogging;

public class ArduinoSubsystem extends SubsystemBase implements ShuffleboardLogging {

	// PIDs
	private final PIDController m_anglePid = new PIDController(ArduinoConstants.kAngleP, ArduinoConstants.kAngleI,
			ArduinoConstants.kAngleD);
	private final PIDController m_distancePid = new PIDController(ArduinoConstants.kDistanceP,
			ArduinoConstants.kDistanceI, ArduinoConstants.kDistanceD);
	// I2C communication
	private final I2C m_wire = new I2C(Port.kOnboard, ArduinoConstants.kAddress);
	// data written to Arduino
	private byte[] m_writeData = new byte[1];
	// data read from Arduino
	private byte[] m_readData = new byte[7];
	private boolean m_targetInView;
	private int m_xValue;
	private int m_distance;
	// PID outputs
	private double m_turnSpeed;
	private double m_driveSpeed;

	/**
	 * Initializes a new instance of the {@link ArduinoSubsystem} class.
	 */
	public ArduinoSubsystem() {
		m_anglePid.setSetpoint(ArduinoConstants.kAngleSetpoint);
		m_anglePid.setTolerance(ArduinoConstants.kAngleTolerance);
		m_distancePid.setSetpoint(ArduinoConstants.kDistanceSetpoint);
		m_distancePid.setTolerance(ArduinoConstants.kDistanceTolerance);
	}

	/**
	 * @return Speed to turn to face target.
	 */
	public double getTurnSpeed() {
		return m_turnSpeed;
	}

	/**
	 * @return Speed to turn to drive towards target.
	 */
	public double getDriveSpeed() {
		return m_driveSpeed;
	}

	/**
	 * @return Whether both PIDs are at their setpoints.
	 */
	public boolean atSetpoint() {
		return m_anglePid.atSetpoint() && m_distancePid.atSetpoint();
	}

	/**
	 * Updates I2C stuff.
	 */
	public void update() {
		read();
		write();
		m_turnSpeed = -m_anglePid.calculate(m_xValue);
		m_driveSpeed = m_distancePid.calculate(m_distance);
	}

	/**
	 * Reads data sent from Arduino.
	 */
	public void read() {
		// read byte array
		m_wire.read(ArduinoConstants.kAddress, m_readData.length, m_readData);
		// set values from array to variables
		m_targetInView = m_readData[ArduinoConstants.kReadTargetInView] == 1;
		m_xValue = 0;
		for (int i : ArduinoConstants.kReadXValue)
			m_xValue += m_readData[i];
		m_distance = 0;
		for (int i : ArduinoConstants.kReadDistance)
			m_distance += m_readData[i];
		// m_xValue = m_readData[1] + m_readData[2] + m_readData[3];
		// m_distance = m_readData[4] + m_readData[5] + m_readData[6];
	}

	/**
	 * Writes data to Arduino.
	 */
	public void write() {
		// write byte array
		m_wire.writeBulk(m_writeData, m_writeData.length);
	}

	/**
	 * @return Whether or not a target is in the camera's view.
	 */
	public boolean getTargetInView() {
		return m_targetInView;
	}

	/**
	 * @return X-value of target in pixels.
	 */
	public int getXValue() {
		return m_xValue;
	}

	/**
	 * @return Distance to the target in inches.
	 */
	public int getDistance() {
		return m_distance;
	}

	public void configureShuffleboard() {
		ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Arduino");
		shuffleboardTab.add("Angle PID", m_anglePid).withSize(1, 2).withPosition(0, 0)
				.withWidget(BuiltInWidgets.kPIDController);
		shuffleboardTab.add("Distance PID", m_distancePid).withSize(1, 2).withPosition(1, 0)
				.withWidget(BuiltInWidgets.kPIDController);
		shuffleboardTab.addBoolean("Target in view", () -> m_targetInView).withSize(1, 1).withPosition(2, 0)
				.withWidget(BuiltInWidgets.kBooleanBox);
		shuffleboardTab.addBoolean("At Setpoint", () -> atSetpoint()).withSize(1, 1).withPosition(2, 1)
				.withWidget(BuiltInWidgets.kBooleanBox);
		shuffleboardTab.addNumber("X Value", () -> m_xValue).withSize(1, 1).withPosition(3, 0)
				.withWidget(BuiltInWidgets.kTextView);
		shuffleboardTab.addNumber("Distance", () -> m_distance).withSize(1, 1).withPosition(3, 1)
				.withWidget(BuiltInWidgets.kTextView);
		shuffleboardTab.addNumber("Turn Speed", () -> m_turnSpeed).withSize(1, 1).withPosition(4, 0)
				.withWidget(BuiltInWidgets.kTextView);
		shuffleboardTab.addNumber("Drive Speed", () -> m_driveSpeed).withSize(1, 1).withPosition(4, 1)
				.withWidget(BuiltInWidgets.kTextView);
	}
}