package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

	private final TalonSRX m_motor = new TalonSRX(IntakeConstants.kMotorPort);

	/**
	 * Initializes a new instance of the {@link IntakeSubsystem} class.
	 */
	public IntakeSubsystem() {
		m_motor.setNeutralMode(NeutralMode.Coast);
		m_motor.enableVoltageCompensation(true);
		m_motor.setInverted(IntakeConstants.kInvert);
	}

	/**
	 * Sets new speed for the intake wheel to spin at.
	 * 
	 * @param speed Percent output.
	 */
	public void setSpeed(double speed) {
		m_motor.set(ControlMode.PercentOutput, speed);
	}
}