package frc.robot.commands.drivecommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightTurnCommand extends CommandBase {

    private final LimelightSubsystem m_limelightSubsystem;
    private final DriveSubsystem m_driveSubsystem;
    private final Supplier<Double> m_turnGoal;
    private final ProfiledPIDController m_turnController = new ProfiledPIDController(LimelightConstants.kTurnP,
            LimelightConstants.kTurnI, LimelightConstants.kTurnD, new Constraints(
                    DriveConstants.kMaxRotSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared));

    /**
     * Use the limelight to reach a desired angle to the powerport
     * 
     * @param limelightSubsystem The limelight subsystem to gather data from
     * @param driveSubsystem     The drivetrain subsystem to be used
     * @param turnGoal           Supplier of the angle setpoint towards the target
     */
    public LimelightTurnCommand(LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem,
            Supplier<Double> turnGoal) {
        m_limelightSubsystem = limelightSubsystem;
        m_driveSubsystem = driveSubsystem;
        m_turnGoal = turnGoal;
        addRequirements(driveSubsystem);
    }

    /**
     * Set the tolerance and goal of the PID
     */
    public void initialize() {
        m_turnController.setTolerance(LimelightConstants.kTurnTolerance);
        m_turnController.setGoal(m_turnGoal.get());
    }

    /**
     * Update the motor outputs
     */
    public void execute() {
        double robotTurnSpeed = m_turnController.calculate(m_limelightSubsystem.getXAngle());
        DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics
                .toWheelSpeeds(new ChassisSpeeds(0, 0, robotTurnSpeed));
        double leftVoltage = DriveConstants.kFeedForward.calculate(wheelSpeeds.leftMetersPerSecond);
        double rightVoltage = DriveConstants.kFeedForward.calculate(wheelSpeeds.rightMetersPerSecond);
        m_driveSubsystem.tankDriveVolts(leftVoltage, rightVoltage);
    }

    /**
     * Stop the drivetrain at the end of the command
     */
    public void end(boolean interrputed) {
        m_driveSubsystem.tankDriveVolts(0, 0);
    }

    /**
     * End the command when the PID is at the setpoint
     */
    public boolean isFinished() {
        return m_turnController.atSetpoint();
    }
}