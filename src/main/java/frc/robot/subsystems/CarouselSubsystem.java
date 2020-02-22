package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CarouselConstants;

public class CarouselSubsystem extends SubsystemBase {

	private final CANSparkMax m_motor = new CANSparkMax(CarouselConstants.kMotorPort, MotorType.kBrushless);
	private final CANEncoder m_encoder = m_motor.getEncoder();
	private final CANPIDController m_pidController = m_motor.getPIDController();

	/**
	 * Initializes a new instance of the {@link CarouselSubsystem} class.
	 */
	public CarouselSubsystem() {
		m_motor.restoreFactoryDefaults();
		m_motor.setInverted(CarouselConstants.kInvert);
		m_motor.setIdleMode(IdleMode.kBrake);
		m_motor.enableVoltageCompensation(12);
		m_motor.setSmartCurrentLimit(CarouselConstants.kSmartCurrentLimit);

		m_pidController.setP(CarouselConstants.kP);
		m_pidController.setI(CarouselConstants.kI);
		m_pidController.setIZone(CarouselConstants.kIz);
		m_pidController.setD(CarouselConstants.kD);
		m_pidController.setFF(CarouselConstants.kFF);
		m_pidController.setOutputRange(CarouselConstants.kMinOutput, CarouselConstants.kMaxOutput);
	}

	/**
	 * @return Position of encoder (rotations).
	 */
	public double getPosition() {
		return m_encoder.getPosition();
	}

	/**
	 * @return Measured velocity of the motor (rpm).
	 */
	public double getVelocity() {
		return m_encoder.getVelocity();
	}

	/**
	 * Set the value of the current encoder position.
	 * 
	 * @param position Value in rotations.
	 */
	public void setPosition(double position) {
		m_encoder.setPosition(position);
	}

	/**
	 * Set new velocity for the carousel to spin at.
	 * 
	 * @param velocity Motor rpm.
	 */
	public void setVelocity(double velocity) {
		if (velocity == 0.0)
			m_motor.set(0.0);
		else
			m_pidController.setReference(velocity, ControlType.kVelocity);
	}

	/**
	 * Zero the encoder position
	 */
	public void resetEncoder() {
		m_encoder.setPosition(0);
	}
}