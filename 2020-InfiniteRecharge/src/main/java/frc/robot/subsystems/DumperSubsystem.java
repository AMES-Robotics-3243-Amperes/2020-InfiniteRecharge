/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
///       Packages and Imports      \\\
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDMotor;
import frc.robot.Constants;
/*\\\     Packages and Imports      ///*/
public class DumperSubsystem extends SubsystemBase {
  
  static CANSparkMax dumpMotors; // Add IDs in Constants.DumperConstants to make more motors here
  private static final double speedMulti = 1;

  public DumperSubsystem() {
    dumpMotors = new CANSparkMax(Constants.IndexerConstants.kIndexCollectID, MotorType.kBrushless);
    
  }

  @Override
  public void periodic() {
    SmartDashboard.getNumber("Dump Current: ", dumpMotors.getOutputCurrent());
    SmartDashboard.getNumber("Dump Voltages: ", dumpMotors.getBusVoltage());
  }

  public static void setMotorSpeed(double speed)
  {
    dumpMotors.set(speed * speedMulti);
  }
}