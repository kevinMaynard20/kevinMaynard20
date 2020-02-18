package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.ShuffleboardLogging;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase implements ShuffleboardLogging {

    private final CANSparkMax m_motor = new CANSparkMax(HoodConstants.kMotorPort, MotorType.kBrushless);
    private final CANEncoder m_encoder = m_motor.getEncoder();
    private final CANPIDController m_PIDController = m_motor.getPIDController();
    private double targetPosition = 0;

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

    public void setPosition(double position) {
        this.targetPosition = position;
        m_PIDController.setReference(position, ControlType.kSmartMotion, HoodConstants.kSlotID, 0);
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    public boolean atSetpoint() {
        return (Math.abs(getPosition() - targetPosition) <= HoodConstants.kAllowedError);
    }

    public void resetEncoder() {
        m_encoder.setPosition(0);
    }

    public double getAngle() {
        // TODO - calculate angle from encoder
        return 0;
    }

    public void updateShuffleboard(ShuffleboardTab shuffleboardTab) {
        shuffleboardTab.add("Position", getPosition()).withSize(2, 2).withPosition(1, 1)
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.add("At setpoint", atSetpoint()).withSize(1, 1).withPosition(3, 1)
                .withWidget(BuiltInWidgets.kBooleanBox);
        shuffleboardTab.add("Angle", getAngle()).withSize(1, 1).withPosition(3, 2).withWidget(BuiltInWidgets.kTextView);
    }
}