/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean TEST_VERSION = false; // When false, disables some troubleshooting SmartDashboard outputs

    public static final class DriveConstants {
        public static final int kLTID = 4;  // CAN
        public static final int kLBID = 3;  // CAN
        public static final int kRTID = 2;  // CAN
        public static final int kRBID = 1;  // CAN

        public static final int kPracLTID = 9;  // Victor PWM
        public static final int kPracLBID = 10; // Victor PWM
        public static final int kPracRTID = 8;  // Victor PWM
        public static final int kPracRBID = 7;  // Victor PWM
        public static final int kPracLEncode3 = 3;  // PWM
        public static final int kPracLEncode4 = 4;  // PWM
        public static final int kPracREncode0 = 0;  // PWM
        public static final int kPracREncode1 = 1;  // PWM

        public static final double DRIVE_SLOW_SPEED = 0.85 * 1;
    }

    public static final class IndexerConstants {
        public static final int kIndexCollectID = 11;   // CAN
        public static final int kIndexShootID = 8;  // CAN
        public static final double BELT_SPEED = 0.75; //* Not even being used
    }
    public static final class ActuatorConstants{
        public static final int actuatorConnectorOne = 1;
        public static final int actuatorConnectorTwo = 2;

    }

    public static class FieldConstants {
        public static double highGoalHeightInches = 8*12 + 2 + (1/4);
    }

    public static class LimelightConstants {
        public static final double PITCH_DEGREES = 9.3;
        // Measurements on limelight lens, 2 measurements each gave the same distance
        //?Change this for preseaon if we want
        public static final double INCHES_TO_FRAME_FRONT = 9 + (9/16);
        public static final double INCHES_TO_GROUND = 25 + (3/16);
    }

    public static final class ClimbingConstant {
        public static final int kClimbAdjID = 10;   // CAN
        public static final int kClimbRID = 12; // CAN
        public static final int kClimbLID = 9;  // CAN

        // Limit switches
        public static final int kBottomWinchLimitID = 3;    // DIO
        public static final int kTopWinchLimitID = 4;   // DIO
        public static final int kLeftLimitID = 0;   // DIO
        public static final int kRightLimitID = 1;  // DIO

    }

    public static final class ControlPanelConstants {
        public static final int kpanelSpinnerID = 5;
        public static final int mechanismLifterID = 10; // Servo ID Don't know yet
    }

    public static final class BallCollectConstants {
        public static final int kSpinID = 7;    // CAN
        public static final int kActuateID = 6; // CAN
        public static final int kCameraServo = 0;   // PWM

        //Limit Switches
        public static final int kBallCollectorRetractedID = 5; //DIO
        public static final int kBallCollectorExtendedID = 6; //DIO
    }

}
