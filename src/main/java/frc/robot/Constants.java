package frc.robot;
//Imports
import edu.wpi.first.math.util.Units;
//EndImports
public class Constants {
    
    // Assigns IDs to Motors
    public static final class DriveConstants {
        public static final int kRightFront = 2;
        public static final int kRightBack = 1;
        public static final int kLeftFront = 18;
        public static final int kLeftBack = 19;
        public static final double kSlowMultiplier = 0.5;
        public static final double kFastMultiplier = 0.8;
    }

    // Assigns IDs to buttons on Xbox Controller
    public static final class XboxConstants {
        public static final int kLeftTrigger = 2;
        public static final int kRightTrigger = 3;
        public static final int kLeftBumper = 5;
        public static final int kRightBumper = 6;
        public static final int kLeftStickX = 0;
        public static final int kLeftStickY = 1;
        public static final int kLeftStickClick = 9;
        public static final int kRightStickX = 4;
        public static final int kRightStickY = 5;
        public static final int kRightStickClick = 10;
        public static final int kAButton = 1;
        public static final int kBButton = 2;
        public static final int kXButton = 3;
        public static final int kYButton = 4;
        public static final int kStartButton = 8;
        public static final int kBackButton = 7;
    }
    //Assigns Values to P and D variables for various tasks (This is for PID loops)
    public static final class PIDConstants {
        public static final double LINEAR_P = -0.1;
        public static final double LINEAR_D = 0.0;
        public static final double ANGULAR_P = 0.035;
        public static final double ANGULAR_I = 0;
        public static final double ANGULAR_D = 0.01;
    }
    // Assigns various values pertaining to vision related commands and subsystems
    public static final class VisionConstants {
        public static final double kCameraHeightMeters = Units.inchesToMeters(12.5); // Height of LimeLight Camera on Robot
        public static final double kTargetHeightMeters = Units.feetToMeters(2.08); // Height of target
        public static final double kGoalRangeMeters = Units.feetToMeters(1);
        public static final double kCameraPitchRadians = Units.degreesToRadians(0);
        public static final double kAngleToleranceDegrees = 5;
    }

    public static final class BalanceConstants {
        public static final double kP = 1/57.;
        public static final double kReverseMultiplier = 1.25;
        public static final double kForwardMultiplier = 5;
    }

    public static final class ArmConstants{
        public static final double kP = 10;
        public static final double kI = 0.0;
        public static final double kD = 0.1;
        public static final int kArmMotor = 10;
        public static final double kGVolts = 0.133;
        public static final double kSVolts = 2;
        public static final double kVVoltSecondPerRad = 1;
        public static final double kAVoltSecondSquaredPerRad = 1;
        public static final double kMaxVelocityRadPerSecond = 1.5;
        public static final double kMaxAccelerationRadPerSecSquared = 2.1;
        public static final double kEncoderDistancePerPulse = 1;
        public static final double kRotationDegree = Units.degreesToRadians(360.0/64.0);
        public static final double kGearRatio = 64;
        public static final double kArmOffsetRads = 0;
        public static final double kArmLength = Units.inchesToMeters(30);
        public static final double kArmMass = Units.lbsToKilograms(10);
        public static final double kArmMinAngle = Units.degreesToRadians(0); //TODO floor angle
        public static final double kArmMaxAngle = Units.degreesToRadians(180);
        public static final double kArmOffsetInDegrees = Units.degreesToRadians(60);
        public static final double kArmLoadingLocation = Units.degreesToRadians(107);
        public static final double kArmRestingLocation = Units.degreesToRadians(15);
        public static final double kArmScoringLocation = Units.degreesToRadians(105);
        public static final double kArmCubeScoringLocation = Units.degreesToRadians(106.5);
        public static final double kArmToFloor = Units.degreesToRadians(190);
        public static final double kFeedForward = 0.133;
        public static final double kArmTolerance = Units.degreesToRadians(2);
    }

    public static final class clawConstants 
    {
        public static final double kClawSpeed = 1;
        /* These next two lines correspond to DIO pins on the roboRIO */
        public static final int kOutLimitSwitch = 0;
        public static final int kInLimitSwitch = 1;
        /* This line is the CAN ID of the claw motor */
        public static final int kClawMotor = 9;
    }


}
