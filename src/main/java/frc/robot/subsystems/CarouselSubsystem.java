package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.ShuffleboardLogging;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CarouselConstants;

public class CarouselSubsystem extends SubsystemBase implements ShuffleboardLogging {

	private final CANSparkMax m_motor = new CANSparkMax(CarouselConstants.kMotorPort, MotorType.kBrushless);
	private final CANEncoder m_encoder = m_motor.getEncoder();
	private final CANPIDController m_pidController = m_motor.getPIDController();

	/**
	 * Initializes a new instance of the {@link CarouselSubsystem} class.
	 */
	public CarouselSubsystem() {
		m_motor.restoreFactoryDefaults();
		m_motor.setSmartCurrentLimit(CarouselConstants.kSmartCurrentLimit);

		m_pidController.setP(CarouselConstants.kP);
		m_pidController.setI(CarouselConstants.kI);
		m_pidController.setD(CarouselConstants.kD);
		m_pidController.setFF(CarouselConstants.kFF);
		m_pidController.setOutputRange(-1.0, 1.0);

		setVelocity(0.0);
	}

	/**
	 * @return Measured velocity of the motor (rpm).
	 */
	public double getVelocity() {
		return m_encoder.getVelocity();
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
	 * @return Position of encoder (rotations).
	 */
	public double getPosition() {
		return m_encoder.getPosition();
	}

	/**
	 * Set the value of the current encoder position.
	 * 
	 * @param position Value in rotations.
	 */
	public void setPosition(double position) {
		m_encoder.setPosition(position);
	}

	public void updateShuffleboard(ShuffleboardTab shuffleboardTab) {
		shuffleboardTab.add("Velocity", getVelocity()).withSize(2, 2).withPosition(1, 1)
				.withWidget(BuiltInWidgets.kGraph);
		shuffleboardTab.add("Current", m_motor.getOutputCurrent()).withSize(2, 2).withPosition(1, 3)
				.withWidget(BuiltInWidgets.kGraph);
		shuffleboardTab.add("PID", m_pidController).withSize(1, 2).withPosition(3, 1)
				.withWidget(BuiltInWidgets.kPIDController);
	}
}