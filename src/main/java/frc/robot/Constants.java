package frc.robot;

public final class Constants {
    public static final class ControllerConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

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
		public static final int[] kReadXValue = {1, 2, 3};
		public static final int[] kReadDistance = {4, 5, 6};
    }

    public static final class CarouselConstants {
		public static final int kMotorPort = 6;
		public static final double kVelocity = 20.83;
		public static final double kRatio = 96.25;
		public static final double kP = 0.000_005;
		public static final double kI = 0.000_000_5;
		public static final double kD = 0.000_5;
		public static final double kFF = 0.0;
	}
    
    public static final class DriveConstants {  // TODO: this is currently for the 2017 robot
        public static final int kMasterLeft = 10;
        public static final int kFollowerLeft1 = 9;
        public static final int kFollowerLeft2 = 8;
    
        public static final int kMasterRight = 4;
        public static final int kFollowerRight1 = 3;
        public static final int kFollowerRight2 = 2;
    }

    public static final class FeederConstants {
		public static final int kMotorPort = 7;
		public static final double kSpeed = 1.0;
	}

    public static final class FlywheelConstants {
        public static final int kFlywheelPort = 22;
		public static final double kP = 0.000_5;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0.000_19;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
		public static final double kMaxRPM = 4000;
	}
}