package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShuffleboardLogging;

public class ClimberSubsystem extends SubsystemBase implements ShuffleboardLogging {

    /**
     * Initializes a new instance of the {@link ClimberSubsystem} class.
     */
    public ClimberSubsystem() {
    }

    /**
     * @param position Setpoint (motor rotations)
     */
    public void setSetpoint(double setpoint) {
    }

    /**
     * @param speed Percent output of the hood
     */
    public void setPercentOutput(Double speed) {
    }

    public void updateShuffleboard(ShuffleboardTab shuffleboardTab) {
    }
}