/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrainSubSystem;

public class LimelightSubsystem extends SubsystemBase {
  /**
   * Creates a new LimelightSubsystem.
   */
  DriveTrainSubSystem m_drive;

  static NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight");
  NetworkTableEntry camMode = table.getEntry("camMode"); // Limelight's operation mode. Always keep at 0!!
  static NetworkTableEntry pipeline = table.getEntry("pipeline"); // Sets limelight's current pipeline *see calibration IP address for more info*
  static NetworkTableEntry tx = table.getEntry("tx"); // What is the x-coordinate?
  static NetworkTableEntry ty = table.getEntry("ty"); // What is the y-coordinate?
  static NetworkTableEntry ta = table.getEntry("ta"); // What is the area?
  static NetworkTableEntry tv = table.getEntry("tv"); // Do you see a valid target "v"?

  static double x = tx.getDouble(0.0);
  static double y = ty.getDouble(0.0);
  static double v = tv.getDouble(0.0);
  static double area = ta.getDouble(0.0);
  boolean target = false;

  //These constants haven't been tuned 1/30/20
  static float KpSteer = 0.000009f;
  static float KpSteer2 = 0.02f;
  static float KpDist = 0.01f;
  static float KpDist2 = 0.2f;
  static float min_command = 0.05f;
  static float refArea = 2.25f;

  public LimelightSubsystem(DriveTrainSubSystem drive) {
    //this class's object "drive" is equal to the DriveTrainSubSystem's object "drive"
    m_drive = drive;
  }

  public static double setDist(){
    double heading_error = x;
    double dist_error = area;
    double dist_adjust = 0.0;
    double maxDistAdjust = 0.8;
    double maxAngAdjust = 1.0;
      if (v == 0.0) {
        dist_adjust = 0.0;
      }

      if (area != 0) {
        dist_error = refArea - dist_error;
        dist_adjust = KpDist * Math.pow(dist_error, 3) + KpDist2 * dist_error;
      }
    return dist_adjust;
  }

  public static double setSteer() {
    // These constants haven't been tuned to the robot!
    double heading_error = x;
    double steer_adjust = 0.0;
    double maxAngAdjust = 1.0;


      if (-.25 < heading_error && heading_error < .25) {
        heading_error = 0.0;
      }

      if (v == 0.0) {
        steer_adjust = 0.5;
      } else if (x > 0 && v != 0) {
        steer_adjust = (KpSteer * Math.pow(heading_error, 3) + KpSteer2 * heading_error) + min_command;
      } else if (x < 0 && v != 0) {
        steer_adjust = (KpSteer * Math.pow(heading_error, 3) + KpSteer2 * heading_error) - min_command;
      }

      steer_adjust = Math.tanh(steer_adjust) * maxAngAdjust;

    return steer_adjust;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(v > 0){
      target = true;
    } else if (v == 0){
      target = false;
    }

    SmartDashboard.putNumber("Lime X: ", x);
    SmartDashboard.putNumber("Lime Y: ", y);
    SmartDashboard.putNumber("Lime Area: ", area);
    SmartDashboard.putNumber("Lime Pipeline: ", (double) pipeline.getNumber(-1)); // Idk what to do here yet
    SmartDashboard.putBoolean("See target?: ", target);
  }
}
