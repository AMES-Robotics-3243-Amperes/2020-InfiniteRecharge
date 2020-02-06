/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kLTID = 2;
        public static final int kLBID = 1;
        public static final int kRTID = 3;
        public static final int kRBID = 4;
    }

    public static final class ShooterConstants {
        public static final int kShootID = 0;   // Don't know ID yet
    }

    public static final class ClimbingConstant {
        public static final int kClimbID = 8;   // Don't know ID yet
    }

    public static final class ControlPanelConstants {
        public static final int kpanelSpinnerID = 5;   //Don't know which port yet, so using random # for now 1/16/20
    }

    public static final class DumperConstants {
        public static final int kDumpID = 7;
    }

    public static final class BallCollectConstants {
        public static final int kBallID = 7; // Don't know which CAN ID this is yet 2/4/20
    }
}
