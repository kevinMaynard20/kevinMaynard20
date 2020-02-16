package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final WPI_TalonSRX m_masterLeft = new WPI_TalonSRX(DriveConstants.kMasterLeftPort);
    private final WPI_TalonSRX m_followerLeftOne = new WPI_TalonSRX(DriveConstants.kFollowerLeftOnePort);
    private final WPI_TalonSRX m_followerLeftTwo = new WPI_TalonSRX(DriveConstants.kFollowerLeftTwoPort);
    private final WPI_TalonSRX m_masterRight = new WPI_TalonSRX(DriveConstants.kMasterRightPort);
    private final WPI_TalonSRX m_followerRightOne = new WPI_TalonSRX(DriveConstants.kFollowerRightOnePort);
    private final WPI_TalonSRX m_followerRightTwo = new WPI_TalonSRX(DriveConstants.kFollowerRightTwoPort);
    private final AHRS m_gyro = new AHRS(DriveConstants.kGyroPort);
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(getHeading()));

    public DriveSubsystem() {
        m_masterLeft.setInverted(DriveConstants.kMasterLeftInvert);
        m_followerLeftOne.setInverted(DriveConstants.kFollowerLeftOneInvert);
        m_followerLeftOne.follow(m_masterLeft);
        m_followerLeftTwo.setInverted(DriveConstants.kFollowerLeftTwoInvert);
        m_followerLeftTwo.follow(m_masterLeft);

        m_masterRight.setInverted(DriveConstants.kMasterRightInvert);
        m_followerRightOne.setInverted(DriveConstants.kFollowerRightOneInvert);
        m_followerRightOne.follow(m_masterRight);
        m_followerRightTwo.setInverted(DriveConstants.kFollowerRightTwoInvert);
        m_followerRightTwo.follow(m_masterRight);

        resetEncoders();
        zeroHeading();
    }

    public void periodic() {
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderPosition(), getRightEncoderPosition());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void arcadeDrive(double straight, double left, double right) {
        tankDrive(straight - left + right, straight + left - right);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_masterLeft.set(leftSpeed);
        m_masterRight.set(rightSpeed);
        m_masterLeft.feed();
        m_masterRight.feed();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_masterLeft.setVoltage(leftVolts);
        m_masterRight.setVoltage(rightVolts);
        m_masterLeft.feed();
        m_masterRight.feed();
    }

    public void resetEncoders() {
        m_masterLeft.setSelectedSensorPosition(0);
        m_masterRight.setSelectedSensorPosition(0);
    }

    public double getLeftEncoderPosition() {
        return m_masterLeft.getSelectedSensorPosition() * Math.PI * DriveConstants.kWheelDiameterMeters
                / DriveConstants.kEncoderEdgesPerRotation;
    }

    public double getRightEncoderPosition() {
        return -m_masterRight.getSelectedSensorPosition() * Math.PI * DriveConstants.kWheelDiameterMeters
                / DriveConstants.kEncoderEdgesPerRotation;
    }

    public double getAverageEncoderDistance() {
        return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
    }

    public double getLeftEncoderVelocity() {
        return m_masterLeft.getSelectedSensorVelocity() * 10 * Math.PI * DriveConstants.kWheelDiameterMeters
                / DriveConstants.kEncoderEdgesPerRotation;
    }

    public double getRightEncoderVelocity() {
        return -m_masterRight.getSelectedSensorVelocity() * 10 * Math.PI * DriveConstants.kWheelDiameterMeters
                / DriveConstants.kEncoderEdgesPerRotation;
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public double getHeading() {
        return m_gyro.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
}