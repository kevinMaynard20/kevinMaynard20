package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.ShuffleboardLogging;

public class ArmSubsystem extends SubsystemBase implements ShuffleboardLogging {

    private final CANSparkMax m_arm = new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
    private final CANEncoder m_encoder = m_arm.getEncoder();
    private final CANPIDController m_PIDController = m_arm.getPIDController();
    private double m_targetPosition = 0;

    /**
     * Initializes a new instance of the {@link FeederSubsystem} class.
     */
    public ArmSubsystem() {
        m_arm.restoreFactoryDefaults();
        m_arm.setInverted(ArmConstants.kInvert);
        m_arm.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_arm.setSmartCurrentLimit(ArmConstants.kSmartCurrentLimit);
        m_PIDController.setOutputRange(-1.0, 1.0);
        m_PIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, ArmConstants.kSlotID);
        m_PIDController.setSmartMotionMaxAccel(ArmConstants.kMaxAcel, ArmConstants.kSlotID);
        m_PIDController.setSmartMotionMaxVelocity(ArmConstants.kMaxVelocity, ArmConstants.kSlotID);
        m_PIDController.setSmartMotionAllowedClosedLoopError(ArmConstants.kAllowedError, ArmConstants.kSlotID);
        m_PIDController.setSmartMotionMinOutputVelocity(ArmConstants.kMinVelocity, ArmConstants.kSlotID);
        m_PIDController.setReference(0, ControlType.kSmartMotion, 0, 0);
        m_PIDController.setP(ArmConstants.kP);
        m_PIDController.setI(ArmConstants.kI);
        m_PIDController.setIZone(ArmConstants.kIz);
        m_PIDController.setD(ArmConstants.kD);
        m_PIDController.setFF(ArmConstants.kFF);
    }

    /**
     * @param position Setpoint (motor rotations)
     */
    public void setSetpoint(double position) {
        m_targetPosition = position;
        m_PIDController.setReference(position, ControlType.kSmartMotion, ArmConstants.kSlotID);
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

    public void updateShuffleboard(ShuffleboardTab shuffleboardTab) {
    }
}