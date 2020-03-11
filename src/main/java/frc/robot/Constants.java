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

		public static final double kAngleP = 0.003;
		public static final double kAngleI = 0.0;
		public static final double kAngleD = 0.0002;
		public static final int kAngleSetpoint = 157;
		// public static final int kAngleTolerance = 10;

		public static final double kDistanceP = 0.01;
		public static final double kDistanceI = 0.0;
		public static final double kDistanceD = 0.0013;
		public static final int kDistanceSetpoint = 20;
		// public static final int kDistanceTolerance = 2;

		public static final int kReadTargetInView = 0;
		public static final int[] kReadXValue = { 1, 2, 3 };
		public static final int[] kReadDistance = { 4, 5, 6 };

		public static final int kWriteMainLEDMode = 0;
		public static final int kWriteMainLEDValue = 1;
		public static final int kWriteShooterLEDMode = 2;
		public static final int kWriteShooterLEDValue = 3;

		public static final class MainLEDModes {
			public static final byte kOff = 0;
			public static final byte kChasing = 1;
		}

		public static final class ShooterLEDModes {
			public static final byte kOff = 0;
			public static final byte kFlywheelPercent = 1;
		}
	}

	public static final class ArmConstants {
		public static final int kMotorPort = 10;
		public static final boolean kInvert = false;
		public static final int kSmartCurrentLimit = 60;
		public static final double kP = .0002;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
		public static final int kSlotID = 0;
		public static final double kMinVelocity = 0;
		public static final double kMaxAcel = 10_000;
		public static final double kMaxVelocity = 5_000;
		public static final double kAllowedError = 0.1;
		public static final double kOutPosition = -46;
		public static final double kInPosition = -5;
		public static final double kMinPosition = -52;
		public static final double kBounceUpPosition = -40;
		public static final double kBounceDownPosition = -50;
		public static final double kBounceTime = .4;
	}

	public static final class CarouselConstants {
		public static final int kMotorPort = 14;
		public static final boolean kInvert = true;
		public static final int kSmartCurrentLimit = 20;

		public static final double kVelP = 0.000001;
		public static final double kVelI = 0;
		public static final double kVelD = 0;
		public static final double kVelIz = 0;
		public static final double kVelFF = 0.000095;

		public static final double kPosP = 0.00010;
		public static final double kPosI = 0;
		public static final double kPosD = 0;
		public static final double kPosIz = 0;
		public static final double kPosFF = 0.0;

		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
		public static final int kSlotID = 0;
		public static final double kMinVelocity = 0;
		public static final double kMaxAcel = 5_000;
		public static final double kMaxVelocity = 8_000;
		public static final double kAllowedError = 0.1;
		public static final double kVelocity = 20;
		public static final double kIntakeVelocity = 30;
		public static final double kJostleVelocity = -65;
		public static final double kRatio = 140;
		public static final double kStartPositionTolerance = 5;
	}

	public static final class ClimberConstants {
		public static final int kMotorPort = 8; // or 9?
		public static final boolean kInvert = false;
		public static final int kSmartCurrentLimit = 60;
		public static final double kP = 0.000_1;
		public static final double kI = 0.0;
		public static final double kD = 0.0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
		public static final int kSlotID = 0;
		public static final double kMinVelocity = 0;
		public static final double kMaxAcel = 40_000;
		public static final double kMaxVelocity = 10_000;
		public static final double kAllowedError = 0.2;
		public static final double kTopSetpoint = -1;
	}

	public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDeadzone = 0.1;
		public static final double kTriggerDeadzone = .05;

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
		public static final int kMasterLeftPort = 4;
		public static final InvertType kMasterLeftInvert = InvertType.None;
		public static final int kFollowerLeftPort = 3;
		public static final InvertType kFollowerLeftInvert = InvertType.None;

		public static final int kMasterRightPort = 5;
		public static final InvertType kMasterRightInvert = InvertType.InvertMotorOutput;
		public static final int kFollowerRightPort = 6;
		public static final InvertType kFollowerRightInvert = InvertType.InvertMotorOutput;

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

		public static final double kTurningMultiplier = .55;
		public static final double kQuickStopThreshold = .2;
		public static final double kQuickStopAlpha = .1;
	}

	public static final class FeederConstants {
		public static final boolean kInvert = false;
		public static final int kMotorPort = 1;
		public static final double kSpeed = 0.75;
	}

	public static final class FlywheelConstants {
		public static final int kMasterPort = 11;
		public static final int kFollowerPort = 13;
		public static final boolean kMasterInvert = false;
		public static final boolean kFollowerInvert = true;
		public static final int kSmartCurrentLimit = 50;
		public static final double kPeakCurrentLimit = 65;
		public static final int kPeakCurrentDurationMillis = 100;
		public static final double kP = 0.002;
		public static final double kI = 0;
		public static final double kD = 0.0;
		public static final double kIz = 0.0;
		public static final double kFF = 0.000_20;
		public static final double kS = 0.217;
		public static final double kV = 0.0583;
		public static final double kA = 0.0787;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
		public static final double kRatio = 2.4;
		public static final double kAllowedErrorPercent = 2;
	}

	public static final class HoodConstants {
		public static final int kMotorPort = 12;
		public static final boolean kInvert = true;
		public static final int kSmartCurrentLimit = 60;
		public static final double kP = 0.000_1;
		public static final double kI = 0.0;
		public static final double kD = 0.0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
		public static final int kSlotID = 0;
		public static final double kMinVelocity = 0;
		public static final double kMaxAcel = 20_000;
		public static final double kMaxVelocity = 10_000;
		public static final double kAllowedError = 0.2;
		public static final double kMinEncoderValue = 0.0;
		public static final double kMaxEncoderValue = 42.0;
		public static final double kMinAngle = 24.36;
		public static final double kMaxAngle = 77.64;
	}

	public static final class IntakeConstants {
		public static final boolean kInvert = true;
		public static final int kMotorPort = 2;
		public static final double kPercentOutput = 0.5;
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

	public static final class LoggingConstants {
		// Arduino, Arm, Carousel, Climber, Drive, Feeder, Flywheel, Hood, Intake,
		// Limelight
		public static final boolean[] kSubsystems = { false, false, false, false, false, false, true, true, false,
				false };
	}

	public enum FieldLocation {
		// WALL(4100, 0, 60, 0, 0), INITLINE(4800, 30, 40, 0, 0), CLOSETRENCH(5900, 40,
		// 20, 0, 0),
		// FARTRENCH(9000, 30, 20, 0, 0);
		
		WALL(4100, 3, 60, 0, 0), TWOFEET(2850, 7, 20, 0, 0), INITLINE(3300, 15, 20, 0, 0),
		CLOSETRENCH(4700, 32, 20, 0, 0), FARTRENCH(6500, 40, 20, 0, 0);

		public final double flywheelSetpoint, hoodSetpoint, carouselSetpoint, distanceGoal, turnGoal;

		private FieldLocation(double flywheelSetpoint, double hoodSetpoint, double carouselSetpoint,
				double distanceGoal, double turnGoal) {
			this.flywheelSetpoint = flywheelSetpoint;
			this.hoodSetpoint = hoodSetpoint;
			this.carouselSetpoint = carouselSetpoint * CarouselConstants.kRatio;
			this.distanceGoal = distanceGoal;
			this.turnGoal = turnGoal;
		}

		public static final FieldLocation fromDistance(double distance) {
			FieldLocation closestDistance = WALL;
			for (FieldLocation fieldLocation : FieldLocation.values()) {
				if (Math.abs(distance - fieldLocation.distanceGoal) < Math
						.abs(distance - closestDistance.distanceGoal)) {
					closestDistance = fieldLocation;
				}
			}
			return closestDistance;
		}

		public static final FieldLocation fromFlywheelSetpoint(double flywheelSetpoint) {
			FieldLocation closestSetpoint = WALL;
			for (FieldLocation fieldLocation : FieldLocation.values()) {
				if (Math.abs(flywheelSetpoint - fieldLocation.flywheelSetpoint) < Math
						.abs(flywheelSetpoint - closestSetpoint.flywheelSetpoint)) {
					closestSetpoint = fieldLocation;
				}
			}
			return closestSetpoint;
		}
	}
}