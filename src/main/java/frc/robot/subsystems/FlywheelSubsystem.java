package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.ShuffleboardLogging;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;

public class FlywheelSubsystem extends SubsystemBase implements ShuffleboardLogging {

    private final CANSparkMax m_neoFlywheelMaster = new CANSparkMax(FlywheelConstants.kMasterPort,
            MotorType.kBrushless);
    private final CANSparkMax m_neoFlywheelFollower = new CANSparkMax(FlywheelConstants.kFollowerPort,
            MotorType.kBrushless);
    private final CANPIDController m_neoController = m_neoFlywheelMaster.getPIDController();
    private final CANEncoder m_neoEncoderMaster = m_neoFlywheelMaster.getEncoder();
    private final CANEncoder m_neoEncoderFollower = m_neoFlywheelFollower.getEncoder();
    private double m_setPoint;

    /**
     * Initializes a new instance of the {@link FlywheelSubsystem} class.
     */
    public FlywheelSubsystem() {
        // Initialize Motors
        m_neoFlywheelMaster.restoreFactoryDefaults();
        m_neoFlywheelMaster.setInverted(FlywheelConstants.kMasterInvert);
        m_neoFlywheelMaster.setIdleMode(IdleMode.kCoast);
        m_neoFlywheelMaster.enableVoltageCompensation(12);
        m_neoFlywheelMaster.setSmartCurrentLimit(FlywheelConstants.kSmartCurrentLimit);
        m_neoFlywheelMaster.setSecondaryCurrentLimit(FlywheelConstants.kPeakCurrentLimit,
                FlywheelConstants.kPeakCurrentDurationMillis);
        m_neoFlywheelMaster.enableSoftLimit(SoftLimitDirection.kReverse, true);

        m_neoFlywheelFollower.restoreFactoryDefaults();
        m_neoFlywheelFollower.setInverted(FlywheelConstants.kMasterInvert);
        m_neoFlywheelFollower.setIdleMode(IdleMode.kCoast);
        m_neoFlywheelFollower.enableVoltageCompensation(12);
        m_neoFlywheelFollower.setSmartCurrentLimit(FlywheelConstants.kSmartCurrentLimit);
        m_neoFlywheelFollower.setSecondaryCurrentLimit(FlywheelConstants.kPeakCurrentLimit,
                FlywheelConstants.kPeakCurrentDurationMillis);
        m_neoFlywheelFollower.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_neoFlywheelFollower.follow(m_neoFlywheelMaster);

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
            m_neoFlywheelMaster.stopMotor();
        } else {
            m_neoController.setReference(setPoint / FlywheelConstants.kRatio, ControlType.kVelocity);
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
        return m_neoEncoderMaster.getVelocity();
    }

    public void updateShuffleboard(ShuffleboardTab shuffleboardTab) {
        shuffleboardTab.add("Setpoint", getSetpoint()).withSize(1, 1).withPosition(1, 1)
                .withWidget(BuiltInWidgets.kTextView);
        shuffleboardTab.add("Velocity Master", m_neoEncoderMaster.getVelocity()).withSize(2, 2).withPosition(1, 2)
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.add("Velocity Master", m_neoEncoderFollower.getVelocity()).withSize(2, 2).withPosition(3, 2)
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.add("Current", m_neoEncoderMaster.getVelocity()).withSize(2, 2).withPosition(1, 4)
                .withWidget(BuiltInWidgets.kGraph);
    }
}
