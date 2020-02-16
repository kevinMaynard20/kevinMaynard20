package frc.robot.commands.drivecommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class VelocityDriveCommand extends CommandBase {

    private final DriveSubsystem m_driveSubsystem;
    private final Supplier<Double> m_speedStraight, m_speedLeft, m_speedRight;

    /**
     * Drive using speed inputs as a velocity percentage out of a maximum velocity
     * 
     * @param DriveSubsystem
     * @param speedStraight  Joystick input for the straight speed
     * @param speedLeft      Joystick input for left speed
     * @param speedRight     Joystick input for the right speed
     */
    public VelocityDriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> speedStraight,
            Supplier<Double> speedLeft, Supplier<Double> speedRight) {
        m_driveSubsystem = driveSubsystem;
        m_speedStraight = speedStraight;
        m_speedLeft = speedLeft;
        m_speedRight = speedRight;
        addRequirements(driveSubsystem);
    }

    public void execute() {
        double speedStraight = Math.abs(m_speedStraight.get()) > ControllerConstants.kDeadzone ? m_speedStraight.get()
                : 0;
        double speedLeft = Math.abs(m_speedLeft.get()) > ControllerConstants.kDeadzone ? m_speedLeft.get() : 0;
        double speedRight = Math.abs(m_speedRight.get()) > ControllerConstants.kDeadzone ? m_speedRight.get() : 0;

        // Calculate robot velocity before calculating Drive wheel speeds
        DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics
                .toWheelSpeeds(new ChassisSpeeds(speedStraight * DriveConstants.kMaxSpeedMetersPerSecond, 0,
                        (speedLeft - speedRight) * DriveConstants.kMaxRotSpeedMetersPerSecond));

        // Use the feedforward to calculate motor voltages to achieve desired wheel
        // speeds
        double leftVoltage = DriveConstants.kFeedForward.calculate(wheelSpeeds.leftMetersPerSecond);
        double rightVoltage = DriveConstants.kFeedForward.calculate(wheelSpeeds.rightMetersPerSecond);
        m_driveSubsystem.tankDriveVolts(leftVoltage, rightVoltage);
    }
}
