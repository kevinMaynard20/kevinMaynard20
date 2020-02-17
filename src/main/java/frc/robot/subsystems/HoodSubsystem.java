package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.ShuffleboardLogging;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase implements ShuffleboardLogging {

    private final CANSparkMax m_motor = new CANSparkMax(HoodConstants.kMotorPort, MotorType.kBrushless);
    private final CANEncoder m_Encoder = m_motor.getEncoder();
    private  CANPIDController m_PIDController = m_motor.getPIDController();
    private double targetPosition = 0;


    /**
     * Initializes a new instance of the {@link HoodSubsystem} class.
     */
    public HoodSubsystem() {
        m_motor.restoreFactoryDefaults();   
        m_motor.setInverted(false);//?
        m_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_motor.setSmartCurrentLimit(HoodConstants.kCurrentLimit);
        m_PIDController.setOutputRange(-1.0, 1.0);
        m_PIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, HoodConstants.KSlotID);
        m_PIDController.setSmartMotionMaxAccel(HoodConstants.kMaxAcel, HoodConstants.KSlotID);
        m_PIDController.setSmartMotionMaxVelocity(HoodConstants.kMaxVelocity, HoodConstants.KSlotID);
        m_PIDController.setSmartMotionAllowedClosedLoopError(HoodConstants.kAllowedError, HoodConstants.KSlotID);
        m_PIDController.setSmartMotionMinOutputVelocity(HoodConstants.kMinVelocity, HoodConstants.KSlotID);
        m_PIDController.setReference(0, ControlType.kSmartMotion, 0, 0);
        m_PIDController.setP(HoodConstants.kPIDP); 
        m_PIDController.setI(HoodConstants.kPIDI);   
        m_PIDController.setIZone(HoodConstants.kPIDIZone);
        m_PIDController.setD(HoodConstants.kPIDD); 
        m_Encoder.setPosition(0);
    }

    public void setPosition(double position) {
        this.targetPosition = position;
        m_PIDController.setReference(position, ControlType.kSmartMotion, HoodConstants.KSlotID, 0);
    }
    public double getPosition(){
        return m_Encoder.getPosition();
    }
    public double getVelocity(){
        return m_Encoder.getVelocity();
    }
    public boolean atSetpoint(){
        return (Math.abs(getPosition()-targetPosition)<=HoodConstants.kAllowedError);
    }
    public void updateShuffleboard(ShuffleboardTab shuffleboardTab) {
        shuffleboardTab.add("Position", getPosition()).withSize(2, 2).withPosition(1, 1)
                .withWidget(BuiltInWidgets.kGraph);
    }
}