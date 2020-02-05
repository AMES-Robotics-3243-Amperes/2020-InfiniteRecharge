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

    }

    public static final class LimeLightConstants {

    }

    public static final class ClimbingConstant {

    }

    public static final class ControlPanelConstants {
        public static final int kMotorCP = 0;   //Don't know which port yet, so using random # for now 1/16/20
    }

    public static final class BallCollectConstants {
        public static final int kBallID = 7; // Don't know which CAN ID this is yet 2/4/20
    }
}
