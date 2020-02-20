/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.DrivetrainPIDSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveTrainSubSystem extends SubsystemBase {

  static SpeedController motorLT;
  static SpeedController motorLB;
  static SpeedController motorRT;
  static SpeedController motorRB;

  static Encoder leftVictorEncode;
  static Encoder rightVictorEncode;
  static CANEncoder leftSparkEncode;
  static CANEncoder rightSparkEncode;

  // New Encoder Objects \\

  // Left and right side drive
  public static SpeedControllerGroup m_leftmotors; // Classifying left side motors
  public static SpeedControllerGroup m_rightmotors; // Classifying right side motors

  static boolean testBot = false;
  static double leftVector = 0.0; // deci multiple of 0.11111 - 0.99999
  static double rightVector = 0.0;

  // m_drive is a combination of both left and right motors
  //private DifferentialDrive m_drive;

  // Encoder data objects
  private static DrivetrainPIDSubsystem m_rightSide;
  private static DrivetrainPIDSubsystem m_leftSide;
  static PIDController m_PIDleft = new PIDController(1, 0, 0);
  static PIDController m_PIDright = new PIDController(1, 0, 0);
  // Command Based code requirement: enabling motors
  public DriveTrainSubSystem() {

    // Checks to see if the robot is the practice robot or competition robot
    if (RobotContainer.isPractice) { // If practice robot, use VictorSPX motors and encoders

      motorLT = new WPI_VictorSPX(Constants.DriveConstants.kPracLTID);
      motorLB = new WPI_VictorSPX(Constants.DriveConstants.kPracLBID);
      motorRT = new WPI_VictorSPX(Constants.DriveConstants.kPracRTID);
      motorRB = new WPI_VictorSPX(Constants.DriveConstants.kPracRBID);

      leftVictorEncode = new Encoder(Constants.DriveConstants.kPracLEncode3, Constants.DriveConstants.kPracLEncode4, false, EncodingType.k4X); // The external encoder on the practice robot
      rightVictorEncode = new Encoder(Constants.DriveConstants.kPracREncode0, Constants.DriveConstants.kPracREncode1, false, EncodingType.k4X);

      m_leftmotors = new SpeedControllerGroup(motorLT, motorLB); // Classifying left side motors
      m_rightmotors = new SpeedControllerGroup(motorRT, motorRB); // Classifying right side motors

      m_rightSide = new DrivetrainPIDSubsystem(m_rightmotors, null, rightVictorEncode);
      m_leftSide = new DrivetrainPIDSubsystem(m_leftmotors, null, leftVictorEncode);

      ((WPI_VictorSPX) motorLB).follow((WPI_VictorSPX) motorLT); // motorLB follows the path of motorLT
      ((WPI_VictorSPX) motorRB).follow((WPI_VictorSPX) motorRT); // motorRB follows the path of motorRT

    } else { // If competition robot, use CANSparkMax motors and encoders
      motorLT = new CANSparkMax(Constants.DriveConstants.kLTID, MotorType.kBrushless);
      motorLB = new CANSparkMax(Constants.DriveConstants.kLBID, MotorType.kBrushless);
      motorRT = new CANSparkMax(Constants.DriveConstants.kRTID, MotorType.kBrushless);
      motorRB = new CANSparkMax(Constants.DriveConstants.kRBID, MotorType.kBrushless);

      leftSparkEncode = ((CANSparkMax) motorLT).getEncoder(); // The built in encoder on the competition robot
      rightSparkEncode = ((CANSparkMax) motorRT).getEncoder();

      m_leftmotors = new SpeedControllerGroup(motorLT, motorLB); // Classifying left side motors
      m_rightmotors = new SpeedControllerGroup(motorRT, motorRB); // Classifying right side motors

      m_rightSide = new DrivetrainPIDSubsystem(m_rightmotors, rightSparkEncode, null);
      m_leftSide = new DrivetrainPIDSubsystem(m_leftmotors, leftSparkEncode, null);

      
      ((CANSparkMax) motorLT).setSmartCurrentLimit(39); // Limits the maximum amps
      ((CANSparkMax) motorLB).setSmartCurrentLimit(39);
      ((CANSparkMax) motorRT).setSmartCurrentLimit(39);
      ((CANSparkMax) motorRB).setSmartCurrentLimit(39);

      
    }

    //m_drive = new DifferentialDrive(m_leftmotors, m_rightmotors);

    m_rightSide.enable(); // Enables the PID loop
    m_leftSide.enable(); // Enables the PID loop  

  }

  public static Encoder getVictorLeft(){
    return leftVictorEncode;
  }
  public static Encoder getVictorRight(){
    return rightVictorEncode;
  }
  public static CANEncoder getSparkLeft(){
    return leftSparkEncode;
  }
  public static CANEncoder getSparkRight(){
    return rightSparkEncode;
  } 

  public static void tankDrive(double varLeft, double varRight, boolean bumperLeft) {
    leftVector = varLeft;
    rightVector = varRight;
    //m_rightBumper = rightBumper;
    if(bumperLeft){
      m_rightSide.setSetpoint(-varRight * 1.5);
      m_leftSide.setSetpoint(varLeft * 1.5);
    } else {
      m_rightSide.setSetpoint(-varRight);
      m_leftSide.setSetpoint(varLeft);
  
    }

  }

  public static void setPosition(double leftSet, double rightSet){
    double motorSpeedLeft; 
    double motorSpeedRight;

    if(leftSparkEncode != null && rightSparkEncode != null){
      motorSpeedLeft = m_PIDleft.calculate(leftSparkEncode.getPosition() , leftSet);
      motorSpeedRight = m_PIDright.calculate(rightSparkEncode.getPosition(), rightSet);

    } else if(leftVictorEncode != null && rightVictorEncode != null){
      motorSpeedLeft = m_PIDleft.calculate(leftVictorEncode.getDistance(), leftSet);
      motorSpeedRight = m_PIDright.calculate(rightVictorEncode.getDistance(), rightSet);
      
    } else{
      motorSpeedLeft = 0;
      motorSpeedRight = 0;
    }
    m_leftSide.setSetpoint(motorSpeedLeft);
    m_leftSide.setSetpoint(motorSpeedRight);
    
  }


  @Override
  public void periodic() {

    SmartDashboard.putNumber("Vector Left: ", leftVector);
    SmartDashboard.putNumber("Vector Right: ", rightVector);

    if (!RobotContainer.isPractice) {
      // Prints speed of encoder
      SmartDashboard.getNumber("VelocityMotorLT: ", ((CANSparkMax) motorLT).getEncoder().getVelocity());
      SmartDashboard.getNumber("VelocityMotorLB: ", ((CANSparkMax) motorLB).getEncoder().getVelocity());
      SmartDashboard.getNumber("VelocityMotorRT: ", ((CANSparkMax) motorRT).getEncoder().getVelocity());
      SmartDashboard.getNumber("VelocityMotorRB: ", ((CANSparkMax) motorRB).getEncoder().getVelocity());

      // Prints current in amps
      SmartDashboard.getNumber("CurrentMotorLT: ", ((CANSparkMax) motorLT).getOutputCurrent());
      SmartDashboard.getNumber("CurrentMotorLB: ", ((CANSparkMax) motorLB).getOutputCurrent());
      SmartDashboard.getNumber("CurrentMotorRT: ", ((CANSparkMax) motorRT).getOutputCurrent());
      SmartDashboard.getNumber("CurrentMotorRB: ", ((CANSparkMax) motorRB).getOutputCurrent());

      // Prints the voltage going into the motor controller
      SmartDashboard.getNumber("CurrentMotorLT: ", ((CANSparkMax) motorLT).getBusVoltage());
      SmartDashboard.getNumber("CurrentMotorLB: ", ((CANSparkMax) motorLB).getBusVoltage());
      SmartDashboard.getNumber("CurrentMotorRT: ", ((CANSparkMax) motorRT).getBusVoltage());
      SmartDashboard.getNumber("CurrentMotorRB: ", ((CANSparkMax) motorRB).getBusVoltage());
  
    }

    // This method will be called once per scheduler run

  }

}
