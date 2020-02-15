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
        public static final int kLTID = 2;
        public static final int kLBID = 1;
        public static final int kRTID = 3;
        public static final int kRBID = 4;

        public static final int kPracLTID = 9;
        public static final int kPracLBID = 10;
        public static final int kPracRTID = 8;
        public static final int kPracRBID = 7;
        public static final int kPracLEncode3 = 3;
        public static final int kPracLEncode4 = 4;
        public static final int kPracREncode0 = 0;
        public static final int kPracREncode1 = 1;
    }

    public static final class IndexerConstants {
        public static final int[] kBeltIDs = { 0 };   // Don't know ID yet
        public static final double BELT_SPEED = 1;
    }

    public static final class ClimbingConstant {
        public static final int kClimbAdjID = 14;   // Don't know ID yet
        public static final int kClimbRID = 12;
        public static final int kClimbLID = 13;
        public static final int kLimitTopID = 7;  // Changed from 1 to 7
        public static final int kLimitBottomID = 6; // Changed from 2 to 6
    }

    public static final class ControlPanelConstants {
        public static final int kpanelSpinnerID = 5;   //Don't know which port yet, so using random # for now 1/16/20
        public static final int mechanismLifterID = 0; // Servo ID
    }

    public static final class DumperConstants {
        public static final int[] kDumpIDs = { 111 }; // Changed from 9 to 111
    }

    public static final class BallCollectConstants {
        public static final int kSpinID = 15; // Don't know which CAN ID this is yet 2/4/20
        public static final int kActuateID = 16; // Don't know which CAN ID this is yet 2/4/20
    }

}
