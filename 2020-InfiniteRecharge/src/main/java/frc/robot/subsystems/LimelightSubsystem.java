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
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubSystem;

public class LimelightSubsystem extends SubsystemBase {
  /**
   * Creates a new LimelightSubsystem.
   */

  static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  static NetworkTableEntry tx = table.getEntry("tx"); // What is the x-coordinate?
  static NetworkTableEntry ty = table.getEntry("ty"); // What is the y-coordinate?
  static NetworkTableEntry ta = table.getEntry("ta"); // What is the area?
  static NetworkTableEntry tv = table.getEntry("tv"); // Do you see a valid target "v"?

  static double x;
  static double y;
  static double v;
  static double area;

  static PIDController m_PIDSteer = new PIDController(1e-2, 1e-9, 1.44e-5);
  static PIDController m_PIDDist = new PIDController(0.15, 0, 7.5e-10);
  
  boolean target = false;
  boolean alignDone = false;

  static double refArea = 0;
  static final int IS_TALL = 5; // Look for the height difference between the loading bay & shooting goal targets

  public LimelightSubsystem() {
    m_PIDSteer.setIntegratorRange(-0.9, 0.9);
    //this class's object "drive" is equal to the DriveTrainSubSystem's object "drive"
  }

  public static void getLimeValues() { //  Gets values or sets a default of 0.
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    v = tv.getDouble(0.0);
    area = ta.getDouble(0.0);  // Size of target in image: from 0-100
  }

  public static double setPIDSteer(){ // Determines amount to steer based on target deviance from center
    if(m_PIDSteer.calculate(x, 0) >= -0.001 || m_PIDSteer.calculate(x, 0) <= 0.001){
      return 0.0;
    } else {
      return m_PIDSteer.calculate(x, 0);  
    }

  }

  public static double setPIDDist(){
    // We may need to get rid of this, because we can just ram into the loading bay wall and gather balls
    if(y >= IS_TALL){
      // Shooting goal's vision target
      refArea = 11.5;
    } else if(y < IS_TALL){
      // Loading bay's vision target
      refArea = 5;
    }

    if(m_PIDDist.calculate(area, refArea) >= -0.001 || m_PIDDist.calculate(area, refArea) <= 0.001){
      return 0.0;
    } else {
      return m_PIDDist.calculate(area, refArea); // Deviance of area from refArea
    }

  }

  /** Returns the distance from the front of the robot frame to the face of the high goal */
  public double getInchesFromHighGoal()
  {
    return (Constants.FieldConstants.highGoalHeightInches - Constants.LimelightConstants.INCHES_TO_GROUND) / (Math.tan(y + Constants.LimelightConstants.PITCH_DEGREES))
      - Constants.LimelightConstants.INCHES_TO_FRAME_FRONT;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    getLimeValues();

    if(setPIDSteer() == 0.0 && setPIDDist() == 0.0){
      alignDone = true;
    } else {
      alignDone = false;
    }

    if(v > 0){
      target = true;
    } else if (v == 0){
      target = false;
    }

    SmartDashboard.putNumber("Lime X: ", x);
    SmartDashboard.putNumber("Lime Y: ", y);
    SmartDashboard.putNumber("Lime Area: ", area);
    SmartDashboard.putBoolean("See target?: ", target);

    SmartDashboard.putBoolean("Limelight Align Done?", alignDone);
    SmartDashboard.putNumber("Est. Target Dist", getInchesFromHighGoal());

  }
}
