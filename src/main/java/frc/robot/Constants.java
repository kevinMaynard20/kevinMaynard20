package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

public final class Constants {
	public static final class ArduinoConstants {
		public static final int kAddress = 0x1;

		public static final double kAngleP = 0.002;
		public static final double kAngleI = 0.0;
		public static final double kAngleD = 0.0002;
		public static final int kAngleSetpoint = 157;
		public static final int kAngleTolerance = 10;

		public static final double kDistanceP = 0.01;
		public static final double kDistanceI = 0.0;
		public static final double kDistanceD = 0.0013;
		public static final int kDistanceSetpoint = 20;
		public static final int kDistanceTolerance = 2;

		public static final int kReadTargetInView = 0;
		public static final int[] kReadXValue = { 1, 2, 3 };
		public static final int[] kReadDistance = { 4, 5, 6 };
	}

	public static final class CarouselConstants {
		public static final int kMotorPort = 6;
		public static final double kVelocity = 10;
		public static final double kRatio = 96.25;
		public static final double kP = 0.000_005;
		public static final double kI = 0.000_000_5;
		public static final double kD = 0.000_5;
		public static final double kFF = 0.0;
		public static final int kSmartCurrentLimit = 20;
	}

	public static final class ClimberConstants {
		public static final int kMotorPort = 6;
		public static final double kTopSetpoint = -1;
	}

	public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDeadzone = 0.1;

		public static final class Axis {
			public static final int kLeftX = 0;
			public static final int kLeftY = 1;
			public static final int kRightX = 2;
			public static final int kLeftTrigger = 3;
			public static final int kRightTrigger = 4;
			public static final int kRightY = 5;
		}

		public static final class Button {
			public static final int kSquare = 1;
			public static final int kX = 2;
			public static final int kCircle = 3;
			public static final int kTriangle = 4;
			public static final int kLeftBumper = 5;
			public static final int kRightBumper = 6;
			public static final int kShare = 9;
			public static final int kOptions = 10;
			public static final int kLeftStick = 11;
			public static final int kRightStick = 12;
			public static final int kPS = 13;
			public static final int kTrackpad = 14;
		}

		public static final class DPad {
			public static final int kUp = 0;
			public static final int kRight = 90;
			public static final int kDown = 180;
			public static final int kLeft = 270;
		}
	}

	public static final class DriveConstants {
		public static final int kMasterLeftPort = 10;
		public static final InvertType kMasterLeftInvert = InvertType.None;
		public static final int kFollowerLeftPort = 9;
		public static final InvertType kFollowerLeftInvert = InvertType.None;

		public static final int kMasterRightPort = 4;
		public static final InvertType kMasterRightInvert = InvertType.None;
		public static final int kFollowerRightPort = 3;
		public static final InvertType kFollowerRightInvert = InvertType.None;

		public static final SPI.Port kGyroPort = SPI.Port.kMXP;
		public static final boolean kGyroReversed = true;

		public static final double ksVolts = .77;
		public static final double kvVoltSecondsPerMeter = 5.84;
		public static final double kaVoltSecondsSquaredPerMeter = .627;
		public static final double kPDriveVel = 1.69;
		public static final double kTrackwidthMeters = 0.713288;
		public static final double kMaxSpeedMetersPerSecond = 2;
		public static final double kMaxAccelerationMetersPerSecondSquared = .6;
		public static final double kMaxRotSpeedMetersPerSecond = 2;
		public static final double kRamseteB = 2;
		public static final double kRamseteZeta = .7;
		public static final double kWheelDiameterMeters = .1524;
		public static final double kEncoderEdgesPerRotation = 4106;

		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
				kTrackwidthMeters);
		public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(DriveConstants.ksVolts,
				DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);
		public static final DifferentialDriveVoltageConstraint kVoltageConstraint = new DifferentialDriveVoltageConstraint(
				DriveConstants.kFeedForward, DriveConstants.kDriveKinematics, 10);
		public static final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
				DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared)
						.setKinematics(DriveConstants.kDriveKinematics)
						.addConstraint(DriveConstants.kVoltageConstraint);
	}

	public static final class FeederConstants {
		public static final int kMotorPort = 7;
		public static final double kSpeed = 1.0;
		public static final double kStartPositionTolerance = 2.5;
	}

	public static final class FlywheelConstants {
		public static final int kMasterPort = 22;
		public static final int kFollowerPort = 6;
		public static final boolean kMasterInvert = true;
		public static final boolean kFollowerInvert = false;
		public static final int kSmartCurrentLimit = 50;
		public static final int kPeakCurrentDurationMillis = 100;
		public static final double kPeakCurrentLimit = 65;
		public static final double kP = 0.000_375;
		public static final double kI = 0;
		public static final double kD = 0.000_03;
		public static final double kIz = 0.0;
		public static final double kFF = 0.000_26;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
		public static final double kMaxRPM = 9600;
		public static final double kRatio = 2.4;
	}

	public static final class HoodConstants {
		public static final int kMotorPort = 5;
		public static final int kSlotID = 0;
		public static final double kMaxAcel = 20_000;
		public static final double kMaxVelocity = 10_000;
		public static final double kAllowedError = 0.2;
		public static final boolean kInvert = false;
		public static final int kSmartCurrentLimit = 60;
		public static final double kP = 0.000_1;
		public static final double kI = 0.0;
		public static final double kD = 0.0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMinVelocity = 0;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
		public static final double kMinEncoderValue = 0.0;
		public static final double kMaxEncoderValue = 42.0;
		public static final double kMinAngle = 24.36;
		public static final double kMaxAngle = 77.64;
	}

	public static final class IntakeConstants {
		public static final int kMotorPort = 0;
		public static final int kArmPort = 0;
		public static final double kIntakeSpeed = 1.0;
		public static final int kSlotID = 0;
		public static final double kMaxAcel = 20_000;
		public static final double kMaxVelocity = 10_000;
		public static final double kAllowedError = 0.2;
		public static final boolean kInvert = false;
		public static final int kSmartCurrentLimit = 60;
		public static final double kP = 0.000_1;
		public static final double kI = 0.0;
		public static final double kD = 0.0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMinVelocity = 0;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
	}

	public static final class LimelightConstants {// TODO - Update PID and camera values
		public static final double kDisP = .016;
		public static final double kDisI = 0;
		public static final double kDisD = 0;
		public static final double kTurnP = 0.022;
		public static final double kTurnI = 0.00001;
		public static final double kTurnD = 0;
		public static final double kTurnTolerance = .1;
		public static final double kDistanceTolerance = .1;
		public static final double kCameraHeight = 27.6;
		public static final double kCameraAngle = 18.43;
		public static final double kTargetHeight = 89.75;
	}

	public enum FieldLocation {
		WALL(-1, -1, -1, -1), INITLINE(-1, -1, -1, -1), CLOSETRENCH(-1, -1, -1, -1), FARTRENCH(-1, -1, -1, -1);

		public final double flywheelSetpoint, hoodSetpoint, distanceGoal, turnGoal;

		private FieldLocation(double flywheelSetpoint, double hoodSetpoint, double distanceGoal, double turnGoal) {
			this.flywheelSetpoint = flywheelSetpoint;
			this.hoodSetpoint = hoodSetpoint;
			this.distanceGoal = distanceGoal;
			this.turnGoal = turnGoal;
		}
	}
}