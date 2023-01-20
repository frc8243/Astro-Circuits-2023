package frc.robot;
//Imports
import com.ctre.phoenix.motorcontrol.StickyFaults;
//End Imports

public class Constants {
    
    // Assigns IDs to Motors
    public static final class DriveConstants {
        public static final int kRightFront = 0;
        public static final int kRightBack = 1;
        public static final int kLeftFront = 2;
        public static final int kLeftBack = 3;
        public static final double SLOW_SPEED_FRACTION = 0.3;
    }

    // Assigns IDs to buttons on Xbox Controller
    public static final class XboxConstants {
        public static final int LEFT_TRIGGER = 2;
        public static final int RIGHT_TRIGGER = 3;
        public static final int LEFT_BUMPER = 5;
        public static final int RIGHT_BUMPER = 6;
        public static final int LEFT_STICK_X = 0;
        public static final int LEFT_STICK_Y = 1;
        public static final int RIGHT_STICK_X = 4;
        public static final int RIGHT_STICK_Y = 5;
        public static final int A_BUTTON = 1;
        public static final int B_BUTTON = 2;
        public static final int X_BUTTON = 3;
        public static final int Y_BUTTON = 4;
        public static final int START_BUTTON = 8;
        public static final int BACK_BUTTON = 7;
    }
}
