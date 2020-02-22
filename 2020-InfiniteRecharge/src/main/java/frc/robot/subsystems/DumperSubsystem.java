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
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
/*\\\     Packages and Imports      ///*/

public class DumperSubsystem extends SubsystemBase {
  
  static CANSparkMax dumpCollect; // Add IDs in Constants.DumperConstants to make more motors here
  //static CANSparkMax dumpShoot;

  static CANEncoder encodeCollect;
  //static CANEncoder encodeShoot;

  static CANPIDController pidCollect;
  //static CANPIDController pidShoot;

  private static final double ballRotation = 2; // We don't know the correct rotations yet
  
  private static final int shootRPM = 1500;
  private static final int MAX_RPM = 5700;

  // NOT YET TUNED TO CORRECT NUMBERS 2/16/20
  double kp = 0.7;
  double ki = 1.5e-3;
  double kd = 5e-8;
  double min = 0.0;
  double max = 0.99;

  static double encodePosition = 0.0;
  static double encodeVelocity = 0.0;

  public DumperSubsystem() {
    dumpCollect = new CANSparkMax(Constants.IndexerConstants.kIndexCollectID, MotorType.kBrushless);
    //dumpShoot = new CANSparkMax(Constants.IndexerConstants.kIndexShootID, MotorType.kBrushless);
    
    encodeCollect = dumpCollect.getEncoder();
    pidCollect = dumpCollect.getPIDController();

    //encodeShoot = dumpShoot.getEncoder();
    //pidShoot = dumpShoot.getPIDController();

    pidCollect.setP(kp);
    pidCollect.setI(ki);
    pidCollect.setD(kd);
    pidCollect.setOutputRange(min, max);

    //pidShoot.setP(kp);
    //pidShoot.setI(ki);
    //pidShoot.setD(kd);
    //pidShoot.setOutputRange(min, max);

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

  public static void setDumpShootSpeed(boolean value)
  {
    //encodeVelocity = encodeShoot.getVelocity() + shootRPM;

    /*if(encodeVelocity > MAX_RPM){
      encodeVelocity = MAX_RPM;
    }*/

    if(value){
      //pidShoot.setReference(encodeVelocity, ControlType.kVelocity);
    } else{
      //dumpShoot.stopMotor();
    }
  }
  
  public static void shootBall(){
    //Make motor dumpShoot spin continously
    //Check for a certain period of time to pass
    //Move dumpCollect dumpCollect to a certain spot
    }
  }

  @Override
  public void periodic() {
    //SmartDashboard.getNumber("Dump Current: ", dumpCollect.getOutputCurrent());
    //SmartDashboard.getNumber("Dump Voltages: ", dumpCollect.getBusVoltage());
  }

}