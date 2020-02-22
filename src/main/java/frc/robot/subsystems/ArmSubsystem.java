package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax m_arm = new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
    private final CANEncoder m_encoder = m_arm.getEncoder();
    private final CANPIDController m_pidController = m_arm.getPIDController();
    private double m_targetPosition = 0;

    /**
     * Initializes a new instance of the {@link ArmSubsystem} class.
     */
    public ArmSubsystem() {
        m_arm.restoreFactoryDefaults();
        m_arm.setInverted(ArmConstants.kInvert);
        m_arm.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_arm.enableVoltageCompensation(12);
        m_arm.setSmartCurrentLimit(ArmConstants.kSmartCurrentLimit);

        m_pidController.setP(ArmConstants.kP);
        m_pidController.setI(ArmConstants.kI);
        m_pidController.setIZone(ArmConstants.kIz);
        m_pidController.setD(ArmConstants.kD);
        m_pidController.setFF(ArmConstants.kFF);
        m_pidController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);

        m_pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, ArmConstants.kSlotID);
        m_pidController.setSmartMotionMaxAccel(ArmConstants.kMaxAcel, ArmConstants.kSlotID);
        m_pidController.setSmartMotionMaxVelocity(ArmConstants.kMaxVelocity, ArmConstants.kSlotID);
        m_pidController.setSmartMotionAllowedClosedLoopError(ArmConstants.kAllowedError, ArmConstants.kSlotID);
        m_pidController.setSmartMotionMinOutputVelocity(ArmConstants.kMinVelocity, ArmConstants.kSlotID);
    }

    /**
     * @param position Setpoint (motor rotations)
     */
    public void setSetpoint(double position) {
        m_targetPosition = position;
        m_pidController.setReference(position, ControlType.kSmartMotion, ArmConstants.kSlotID);
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
     * @return Whether the arm is at the setpoint
     */
    public boolean atSetpoint() {
        return (Math.abs(getPosition() - m_targetPosition) <= ArmConstants.kAllowedError);
    }

    /**
     * Zero the encoder position
     */
    public void resetEncoder() {
        m_encoder.setPosition(0);
    }
}