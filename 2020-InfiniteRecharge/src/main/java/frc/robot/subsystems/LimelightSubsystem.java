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

import frc.robot.subsystems.DriveTrainSubSystem;

public class LimelightSubsystem extends SubsystemBase {
  /**
   * Creates a new LimelightSubsystem.
   */
  DriveTrainSubSystem m_drive;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight");
  NetworkTableEntry camMode = table.getEntry("camMode");  //Limelight's operation mode. Always keep at 0!!
  NetworkTableEntry pipeline = table.getEntry("pipeline");  //Sets limelight's current pipeline *see calibration IP address for more info*
  NetworkTableEntry tx = table.getEntry("tx");  //What is the x-coordinate?
  NetworkTableEntry ty = table.getEntry("ty");  //What is the y-coordinate?
  NetworkTableEntry ta = table.getEntry("ta");  //What is the area?
  NetworkTableEntry tv = table.getEntry("tv");  //Do you see a valid target "v"?

  double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double v = tv.getDouble(0.0);
  double area = ta.getDouble(0.0);

  //These constants haven't been tuned 1/30/20
  float KpSteer = 0;
  float KpSteer2 = 0;
  float KpDist = 0;
  float KpDist2 = 0;
  float min_command = 0;
  float refArea = 0;

  public LimelightSubsystem(DriveTrainSubSystem drive) {
    m_drive = drive;  //this class's object "drive" is equal to the DriveTrainSubSystem's object "drive"
  }

  public void setVision(boolean value){
    double heading_error = x;
    double steer_adjust = 0.0;
    double dist_error = area;
    double dist_adjust = 0.0;
    double maxDistAdjust = 0.0;
    double maxAngAdjust = 1.0;

    if(value){

      if(-.25 < heading_error && heading_error < .25){
        heading_error = 0.0;
      }

      if(v == 0.0){
        steer_adjust = 0.5;
      }else if(x > 0 && v != 0){
        steer_adjust = (KpSteer * Math.pow(heading_error, 3) + KpSteer2 * heading_error) + min_command;
      }else if(x < 0 && v != 0){
        steer_adjust = (KpSteer * Math.pow(heading_error, 3) + KpSteer2 * heading_error) - min_command;
      }

      if(v == 0.0){
        dist_adjust = 0.0;
      }

      if(area != 0){
        dist_error = refArea - dist_error;
        dist_adjust = KpDist * Math.pow(dist_error, 3) + KpDist2 * dist_error;
      }

      steer_adjust = Math.tanh(steer_adjust) * maxAngAdjust;

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
