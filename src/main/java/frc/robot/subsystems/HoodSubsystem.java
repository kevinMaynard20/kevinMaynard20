package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.ShuffleboardLogging;

public class HoodSubsystem extends SubsystemBase implements ShuffleboardLogging {

    private final CANSparkMax m_motor = new CANSparkMax(HoodConstants.kMotorPort, MotorType.kBrushless);
    private final CANEncoder m_encoder = m_motor.getEncoder();
    private final CANPIDController m_PIDController = m_motor.getPIDController();
    private double m_targetPosition = 0;

    /**
     * Initializes a new instance of the {@link HoodSubsystem} class.
     */
    public HoodSubsystem() {
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(HoodConstants.kInvert);
        m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_motor.setSmartCurrentLimit(HoodConstants.kSmartCurrentLimit);
        m_PIDController.setOutputRange(-1.0, 1.0);
        m_PIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, HoodConstants.kSlotID);
        m_PIDController.setSmartMotionMaxAccel(HoodConstants.kMaxAcel, HoodConstants.kSlotID);
        m_PIDController.setSmartMotionMaxVelocity(HoodConstants.kMaxVelocity, HoodConstants.kSlotID);
        m_PIDController.setSmartMotionAllowedClosedLoopError(HoodConstants.kAllowedError, HoodConstants.kSlotID);
        m_PIDController.setSmartMotionMinOutputVelocity(HoodConstants.kMinVelocity, HoodConstants.kSlotID);
        m_PIDController.setReference(0, ControlType.kSmartMotion, 0, 0);
        m_PIDController.setP(HoodConstants.kP);
        m_PIDController.setI(HoodConstants.kI);
        m_PIDController.setIZone(HoodConstants.kIz);
        m_PIDController.setD(HoodConstants.kD);
        m_PIDController.setFF(HoodConstants.kFF);

        resetEncoder();
    }

    /**
     * @param position Setpoint (motor rotations)
     */
    public void setSetpoint(double position) {
        m_targetPosition = position;
        m_PIDController.setReference(position, ControlType.kSmartMotion, HoodConstants.kSlotID, 0);
    }

    /**
     * @return Current position (motor rotations)
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
        return (Math.abs(getPosition() - m_targetPosition) <= HoodConstants.kAllowedError);
    }

    /**
     * Zero the encoder position
     */
    public void resetEncoder() {
        m_encoder.setPosition(0);
    }

    /**
     * @return The angle of the hood from horizontal
     */
    public double getAngle() {
        return getPosition() * ((HoodConstants.kMaxAngle - HoodConstants.kMinAngle)
                / (HoodConstants.kMaxEncoderValue - HoodConstants.kMinEncoderValue)) + HoodConstants.kMinAngle;
    }

    /**
     * @param speed Percent output of the hood
     */
    public void setPercentOutput(Double speed) {
        m_motor.set(speed);
    }

    public void updateShuffleboard(ShuffleboardTab shuffleboardTab) {
        shuffleboardTab.add("Position", getPosition()).withSize(2, 2).withPosition(0, 0)
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.add("At setpoint", atSetpoint()).withSize(1, 1).withPosition(2, 0)
                .withWidget(BuiltInWidgets.kBooleanBox);
        shuffleboardTab.add("Angle", getAngle()).withSize(1, 1).withPosition(2, 1).withWidget(BuiltInWidgets.kTextView);
    }
}