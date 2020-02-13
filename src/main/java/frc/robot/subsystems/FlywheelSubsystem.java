package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;

public class FlywheelSubsystem extends SubsystemBase {

    private final CANSparkMax m_neoFlywheel = new CANSparkMax(FlywheelConstants.kFlywheelPort, MotorType.kBrushless);
    private final CANPIDController m_neoController = m_neoFlywheel.getPIDController();
    private final CANEncoder m_neoEncoder = m_neoFlywheel.getEncoder();
    private double m_setPoint;

    /**
     * Initializes a new instance of the {@link FlywheelSubsystem} class.
     */
    public FlywheelSubsystem() {
        // Initialize Motors
        m_neoFlywheel.restoreFactoryDefaults();
        m_neoFlywheel.setInverted(true);
        m_neoFlywheel.setIdleMode(IdleMode.kBrake);
        m_neoFlywheel.enableVoltageCompensation(12);
        m_neoFlywheel.setSmartCurrentLimit(FlywheelConstants.kSmartCurrentLimit);
        m_neoFlywheel.setSecondaryCurrentLimit(FlywheelConstants.kPeakCurrentLimit,
                FlywheelConstants.kPeakCurrentDurationMillis);
        m_neoFlywheel.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_neoController.setP(FlywheelConstants.kP);
        m_neoController.setI(FlywheelConstants.kI);
        m_neoController.setD(FlywheelConstants.kD);
        m_neoController.setIZone(FlywheelConstants.kIz);
        m_neoController.setFF(FlywheelConstants.kFF);
        m_neoController.setOutputRange(FlywheelConstants.kMinOutput, FlywheelConstants.kMaxOutput);
    }

    /**
     * Sets target speed for flywheel.
     * 
     * @param setPoint Target velocity (rpm).
     */
    public void setSetpoint(double setPoint) {
        m_setPoint = setPoint;
        if (setPoint == 0) {
            m_neoFlywheel.stopMotor();
        } else {
            m_neoController.setReference(setPoint, ControlType.kVelocity);
        }
    }

    /**
     * @return Current setpoint.
     */
    public double getSetpoint() {
        return m_setPoint;
    }

    /**
     * @return Measured velocity.
     */
    public double getVelocity() {
        return m_neoEncoder.getVelocity();
    }
}
