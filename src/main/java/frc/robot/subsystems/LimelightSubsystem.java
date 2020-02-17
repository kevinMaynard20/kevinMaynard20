package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.ShuffleboardLogging;

public class LimelightSubsystem extends SubsystemBase implements ShuffleboardLogging {

    private final NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");;
    private boolean isTargetVisible;
    private double xAngle, yAngle, distance;

    public LimelightSubsystem() {
    }

    public void periodic() {
        isTargetVisible = m_limelightTable.getEntry("tv").getDouble(0) == 1;
        xAngle = m_limelightTable.getEntry("tx").getDouble(0);
        yAngle = m_limelightTable.getEntry("ty").getDouble(0);
        distance = (LimelightConstants.kTargetHeight - LimelightConstants.kCameraHeight)
                / (Math.tan(Math.toRadians(LimelightConstants.kCameraAngle + getYAngle())));
    }

    public boolean isTargetVisible() {
        return isTargetVisible;
    }

    public double getXAngle() {
        return xAngle;
    }

    public double getYAngle() {
        return yAngle;
    }

    public double getDistance() {
        return distance;
    }

    public void turnOnLight() {
        m_limelightTable.getEntry("ledmode").setNumber(1);
    }

    public void turnOffLight() {
        m_limelightTable.getEntry("ledmode").setNumber(0);
    }

    public boolean isLightOn() {
        return m_limelightTable.getEntry("ledmode").getDouble(0) == 1;
    }

    public void updateShuffleboard(ShuffleboardTab shuffleboardTab) {
        shuffleboardTab.add("X Angle", getXAngle()).withSize(1, 1).withPosition(1, 1)
                .withWidget(BuiltInWidgets.kTextView);
        shuffleboardTab.add("Distance", getDistance()).withSize(1, 1).withPosition(2, 1)
                .withWidget(BuiltInWidgets.kTextView);
        shuffleboardTab.add("Target Visible", isTargetVisible()).withSize(1, 1).withPosition(1, 2)
                .withWidget(BuiltInWidgets.kBooleanBox);
        shuffleboardTab.add("Light On", isLightOn()).withSize(1, 1).withPosition(2, 2)
                .withWidget(BuiltInWidgets.kBooleanBox);
    }
}
