package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TrajectoryFollow extends RamseteCommand {

    private final DriveSubsystem m_driveSubsystem;

    public TrajectoryFollow(DriveSubsystem driveSubsystem, Trajectory trajectory) {
        super(trajectory, driveSubsystem::getPose, new RamseteController(), DriveConstants.kFeedForward,
                DriveConstants.kDriveKinematics, driveSubsystem::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
                driveSubsystem::tankDriveVolts, driveSubsystem);
        m_driveSubsystem = driveSubsystem;
    }

    public void end(boolean interrupted) {
        m_driveSubsystem.tankDriveVolts(0, 0);
    }
}