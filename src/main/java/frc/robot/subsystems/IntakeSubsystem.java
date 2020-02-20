package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.ShuffleboardLogging;

public class IntakeSubsystem extends SubsystemBase implements ShuffleboardLogging {

	private final TalonSRX m_motor = new TalonSRX(IntakeConstants.kMotorPort);

	/**
	 * Initializes a new instance of the {@link IntakeSubsystem} class.
	 */
	public IntakeSubsystem() {
	}

	/**
	 * Sets new speed for the intake wheel to spin at.
	 * 
	 * @param speed Percent output.
	 */
	public void setSpeed(double speed) {
		m_motor.set(ControlMode.PercentOutput, speed);
	}

	public void updateShuffleboard(ShuffleboardTab shuffleboardTab) {
		shuffleboardTab.add("Intake Output Percentage", m_motor.getMotorOutputPercent()).withSize(2, 2)
				.withPosition(0, 0).withWidget(BuiltInWidgets.kGraph);
	}
}