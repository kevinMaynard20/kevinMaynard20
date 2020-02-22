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
    private final WPI_TalonSRX m_followerLeft = new WPI_TalonSRX(DriveConstants.kFollowerLeftPort);
    private final WPI_TalonSRX m_masterRight = new WPI_TalonSRX(DriveConstants.kMasterRightPort);
    private final WPI_TalonSRX m_followerRight = new WPI_TalonSRX(DriveConstants.kFollowerRightPort);
    private final AHRS m_gyro = new AHRS(DriveConstants.kGyroPort);
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(getHeading()));

    /**
     * Initializes a new instance of the {@link DriveSubsystem} class.
     */
    public DriveSubsystem() {
        m_masterLeft.setInverted(DriveConstants.kMasterLeftInvert);
        m_followerLeft.setInverted(DriveConstants.kFollowerLeftInvert);
        m_followerLeft.follow(m_masterLeft);

        m_masterRight.setInverted(DriveConstants.kMasterRightInvert);
        m_followerRight.setInverted(DriveConstants.kFollowerRightInvert);
        m_followerRight.follow(m_masterRight);

        resetEncoders();
        zeroHeading();
    }

    /**
     * Update odometry
     */
    public void periodic() {
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderPosition(), getRightEncoderPosition());
    }

    /**
     * @return The left encoder position (meters)
     */
    public double getLeftEncoderPosition() {
        return -m_masterLeft.getSelectedSensorPosition() * Math.PI * DriveConstants.kWheelDiameterMeters
                / DriveConstants.kEncoderEdgesPerRotation;
    }

    /**
     * @return The right encoder position (meters)
     */
    public double getRightEncoderPosition() {
        return m_masterRight.getSelectedSensorPosition() * Math.PI * DriveConstants.kWheelDiameterMeters
                / DriveConstants.kEncoderEdgesPerRotation;
    }

    /**
     * @return The average encoder distance of both encoders (meters)
     */
    public double getAverageEncoderDistance() {
        return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
    }

    /**
     * @return The velocity of the left encoder (meters/s)
     */
    public double getLeftEncoderVelocity() {
        return -m_masterLeft.getSelectedSensorVelocity() * 10 * Math.PI * DriveConstants.kWheelDiameterMeters
                / DriveConstants.kEncoderEdgesPerRotation;
    }

    /**
     * @return The velocity of the right encoder (meters/s)
     */
    public double getRightEncoderVelocity() {
        return m_masterRight.getSelectedSensorVelocity() * 10 * Math.PI * DriveConstants.kWheelDiameterMeters
                / DriveConstants.kEncoderEdgesPerRotation;
    }

    /**
     * @return Pose of the robot
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * @return Wheel speeds of the robot
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
    }

    /**
     * @return The heading of the gyro (degrees)
     */
    public double getHeading() {
        return m_gyro.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * @return The rate of the gyro turn (deg/s)
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Sets both encoders to 0
     */
    public void resetEncoders() {
        m_masterLeft.setSelectedSensorPosition(0);
        m_masterRight.setSelectedSensorPosition(0);
    }

    /**
     * Reset the heading of the gyro
     */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * @param pose Pose to set the robot to
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * @param straight Straight percent output
     * @param left     Left percent output
     * @param right    Right percent output
     */
    public void arcadeDrive(double straight, double left, double right) {
        tankDrive(straight - left + right, straight + left - right);
    }

    /**
     * @param leftSpeed  Left motors percent output
     * @param rightSpeed Right motors percent output
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_masterLeft.set(leftSpeed);
        m_masterRight.set(rightSpeed);
        m_masterLeft.feed();
        m_masterRight.feed();
    }

    /**
     * @param leftVolts  Left motors desired voltage
     * @param rightVolts Right motors desired voltage
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_masterLeft.setVoltage(leftVolts);
        m_masterRight.setVoltage(rightVolts);
        m_masterLeft.feed();
        m_masterRight.feed();
    }
}