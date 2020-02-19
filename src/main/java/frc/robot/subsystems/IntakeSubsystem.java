package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.ShuffleboardLogging;

public class IntakeSubsystem extends SubsystemBase implements ShuffleboardLogging {

	private final TalonSRX m_motor = new TalonSRX(IntakeConstants.kMotorPort);
	private final CANSparkMax m_arm = new CANSparkMax(IntakeConstants.kArmPort, MotorType.kBrushless);
	private final CANEncoder m_encoder = m_arm.getEncoder();
	private final CANPIDController m_PIDController = m_arm.getPIDController();
	private double m_targetPosition = 0;

	/**
	 * Initializes a new instance of the {@link FeederSubsystem} class.
	 */
	public IntakeSubsystem() {
		m_arm.restoreFactoryDefaults();
		m_arm.setInverted(IntakeConstants.kInvert);
		m_arm.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_arm.setSmartCurrentLimit(IntakeConstants.kSmartCurrentLimit);
		m_PIDController.setOutputRange(-1.0, 1.0);
		m_PIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, IntakeConstants.kSlotID);
		m_PIDController.setSmartMotionMaxAccel(IntakeConstants.kMaxAcel, IntakeConstants.kSlotID);
		m_PIDController.setSmartMotionMaxVelocity(IntakeConstants.kMaxVelocity, IntakeConstants.kSlotID);
		m_PIDController.setSmartMotionAllowedClosedLoopError(IntakeConstants.kAllowedError, IntakeConstants.kSlotID);
		m_PIDController.setSmartMotionMinOutputVelocity(IntakeConstants.kMinVelocity, IntakeConstants.kSlotID);
		m_PIDController.setReference(0, ControlType.kSmartMotion, 0, 0);
		m_PIDController.setP(IntakeConstants.kP);
		m_PIDController.setI(IntakeConstants.kI);
		m_PIDController.setIZone(IntakeConstants.kIz);
		m_PIDController.setD(IntakeConstants.kD);
		m_PIDController.setFF(IntakeConstants.kFF);
	}

	/**
	 * Sets new speed for the feeder wheel to spin at.
	 * 
	 * @param speed Percent output.
	 */
	public void setSpeed(double speed) {
		m_motor.set(ControlMode.PercentOutput, speed);
	}

	/**
	 * @param position Setpoint (motor rotations)
	 */
	public void setSetpoint(double position) {
		m_targetPosition = position;
		m_PIDController.setReference(position, ControlType.kSmartMotion, IntakeConstants.kSlotID);
	}

	/**
	 * @return Current arm position (motor rotations)
	 */
	public double getPosition() {
		return m_encoder.getPosition();
	}

	/**
	 * @return Current velocity (motor rotations/s)
	 */
	public double getVelocity() {
		return m_encoder.getVelocity();
	}

	/**
	 * @return Whether the hood is at the setpoint
	 */
	public boolean atSetpoint() {
		return (Math.abs(getPosition() - m_targetPosition) <= IntakeConstants.kAllowedError);
	}

	/**
	 * Zero the encoder position
	 */
	public void resetEncoder() {
		m_encoder.setPosition(0);
	}

	public void updateShuffleboard(ShuffleboardTab shuffleboardTab) {
		shuffleboardTab.add("Intake Output Percentage", m_motor.getMotorOutputPercent()).withSize(2, 2)
				.withPosition(0, 0).withWidget(BuiltInWidgets.kGraph);
		shuffleboardTab.add("Arm Output Percentage", m_motor.getMotorOutputPercent()).withSize(2, 2).withPosition(2, 0)
				.withWidget(BuiltInWidgets.kGraph);
		shuffleboardTab.add("Arm Setpoint", m_targetPosition).withSize(1, 1).withPosition(0, 2)
				.withWidget(BuiltInWidgets.kTextView);
		shuffleboardTab.add("Arm at setpoint", atSetpoint()).withSize(1, 1).withPosition(1, 2)
				.withWidget(BuiltInWidgets.kBooleanBox);
	}
}