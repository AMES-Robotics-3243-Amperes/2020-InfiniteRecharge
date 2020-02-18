/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
///       Packages and Imports      \\\
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDMotor;
import frc.robot.Constants;
/*\\\     Packages and Imports      ///*/
public class DumperSubsystem extends SubsystemBase {
  
  static CANSparkMax dumpCollect; // Add IDs in Constants.DumperConstants to make more motors here
  static CANSparkMax dumpShoot;
  static CANEncoder encodeCollect;
  static CANPIDController pidCollect;
  private static final double speedMulti = 1;
  private static final double ballRotation = 2; // We don't know the correct rotations yet

  // NOT YET TUNED TO CORRECT NUMBERS 2/16/20
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double min = 0.0;
  double max = 0.99;

  static double encodePosition = 0.0;

  public DumperSubsystem() {
    dumpCollect = new CANSparkMax(Constants.IndexerConstants.kIndexCollectID, MotorType.kBrushless);
    dumpShoot = new CANSparkMax(Constants.IndexerConstants.kIndexShootID, MotorType.kBrushless);
    
    encodeCollect = dumpCollect.getEncoder();
    pidCollect = dumpCollect.getPIDController();

    pidCollect.setP(kp);
    pidCollect.setI(ki);
    pidCollect.setD(kd);
    pidCollect.setOutputRange(min, max);
  }

  public static void setDumpCollectSpeed(boolean value)
  {
    encodePosition = encodeCollect.getPosition() + ballRotation;
    
    if(value){
      pidCollect.setReference(encodePosition, ControlType.kPosition);
    } else {
      dumpCollect.stopMotor();
    }
    
  }

  public static void setDumpShootSpeed(boolean value){
    if(value){
      dumpShoot.set(0.8); // Let's see if 80% is enough for the high goal
    } else{
      dumpShoot.stopMotor();
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.getNumber("Dump Current: ", dumpCollect.getOutputCurrent());
    SmartDashboard.getNumber("Dump Voltages: ", dumpCollect.getBusVoltage());
  }

}