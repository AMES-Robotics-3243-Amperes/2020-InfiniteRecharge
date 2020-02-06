/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
///       Packages and Imports      \\\
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDMotor;
import frc.robot.Constants;
/*\\\     Packages and Imports      ///*/
public class DumperSubsystem extends SubsystemBase {
  
  VictorSPX[] dumpMotors; // Add IDs in Constants.DumperConstants to make more motors here
  private final double speedMulti = 1;

  public DumperSubsystem() {
    dumpMotors = new VictorSPX[Constants.DumperConstants.kDumpIDs.length];
    for(int i=0; i<dumpMotors.length; i++)
    {
      dumpMotors[i] = new VictorSPX(Constants.DumperConstants.kDumpIDs[i]);
    }
  }

  @Override
  public void periodic() {
    
  }

  public void setMotorSpeed(double speed)
  {
    for(VictorSPX motor : dumpMotors)
    {
      motor.set(ControlMode.Velocity, speed * speedMulti);
    }
  }
}