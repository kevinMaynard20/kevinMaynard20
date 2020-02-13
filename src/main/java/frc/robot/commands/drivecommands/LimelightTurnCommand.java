package frc.robot.commands.drivecommands;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightTurnCommand extends CommandBase {

    private final LimelightSubsystem m_limelightSubsystem;
    private final DriveSubsystem m_drivetrainSubsystem;
    private final Supplier<Double> m_turnGoal;
    private final ProfiledPIDController m_turnController = new ProfiledPIDController(LimelightConstants.kTurnP,
            LimelightConstants.kTurnI, LimelightConstants.kTurnD, new Constraints(
                    DriveConstants.kMaxRotSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared));

    public LimelightTurnCommand(LimelightSubsystem limelightSubsystem, DriveSubsystem drivetrainSubsystem,
            Supplier<Double> turnGoal, Supplier<Double> distanceGoal) {
        m_limelightSubsystem = limelightSubsystem;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_turnGoal = turnGoal;
        addRequirements(drivetrainSubsystem);
    }

    public void initialize() {
        m_turnController.setTolerance(LimelightConstants.kTurnTolerance);
        m_turnController.setGoal(m_turnGoal.get());
    }

    public void execute() {
        double robotTurnSpeed = m_turnController.calculate(m_limelightSubsystem.getXAngle());
        DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics
                .toWheelSpeeds(new ChassisSpeeds(0, 0, robotTurnSpeed));
        double leftVoltage = DriveConstants.kFeedForward.calculate(wheelSpeeds.leftMetersPerSecond);
        double rightVoltage = DriveConstants.kFeedForward.calculate(wheelSpeeds.rightMetersPerSecond);
        m_drivetrainSubsystem.tankDriveVolts(leftVoltage, rightVoltage);
    }

    public void end(boolean interrputed) {
        m_drivetrainSubsystem.tankDriveVolts(0, 0);
    }

    public boolean isFinished() {
        return m_turnController.atSetpoint();
    }
}