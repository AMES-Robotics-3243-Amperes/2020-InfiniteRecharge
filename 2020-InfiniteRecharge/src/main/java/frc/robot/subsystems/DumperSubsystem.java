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
  
  CANSparkMax[] dumpMotors; // Add IDs in Constants.DumperConstants to make more motors here
  private final double speedMulti = 1;

  public DumperSubsystem() {
    dumpMotors = new CANSparkMax[Constants.DumperConstants.kDumpIDs.length];
    for(int i=0; i<dumpMotors.length; i++)
    {
      dumpMotors[i] = new CANSparkMax(Constants.DumperConstants.kDumpIDs[i], MotorType.kBrushless);
      dumpMotors[i].setSmartCurrentLimit(30);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.getNumber("Dump Current: ", dumpMotors[0].getOutputCurrent());
    for(CANSparkMax motor : dumpMotors){
      SmartDashboard.getNumber("Dump Voltages: ", motor.getBusVoltage());
    }
  }

  public void setMotorSpeed(double speed)
  {
    for(CANSparkMax motor : dumpMotors)
    {
      motor.set(speed * speedMulti);
    }
  }
}