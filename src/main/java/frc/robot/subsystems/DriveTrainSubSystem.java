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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.DrivetrainPIDSubsystem;
import frc.robot.util.JoystUtil;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DigitalInput;


public class DriveTrainSubSystem extends SubsystemBase {

  static SpeedController motorLT;
  static SpeedController motorLB;
  static SpeedController motorRT;
  static SpeedController motorRB;

  // New Encoder Objects \\
  static Encoder leftVictorEncode;
  static Encoder rightVictorEncode;
  static CANEncoder leftSparkEncode;
  static CANEncoder rightSparkEncode;

  // Left and right side drive
  public static SpeedControllerGroup m_leftmotors; // Classifying left side motors
  public static SpeedControllerGroup m_rightmotors; // Classifying right side motors
  static boolean testBot = false;
  static double leftVector = 0.0; // deci multiple of 0.11111 - 0.99999
  static double rightVector = 0.0;
  static double timeAtLastTankDrive = -1;
  static final double SMOOTH_DECELERATION_COEFF = 2.5;

  // Encoder data objects
  private static DrivetrainPIDSubsystem m_rightSide;
  private static DrivetrainPIDSubsystem m_leftSide;
  static PIDController m_PIDleft = new PIDController(0.045, 0, 0.009);
  static PIDController m_PIDright = new PIDController(0.045, 0, 0.009);

  // Command Based code requirement: enabling motors
  public DriveTrainSubSystem() {

    // Checks to see if the robot is the practice robot or competition robot
    if (RobotContainer.isPractice) { // If practice robot, use VictorSPX motors and encoders

      /* motorLT = new WPI_VictorSPX(Constants.DriveConstants.kPracLTID);
      motorLB = new WPI_VictorSPX(Constants.DriveConstants.kPracLBID);
      motorRT = new WPI_VictorSPX(Constants.DriveConstants.kPracRTID);
      motorRB = new WPI_VictorSPX(Constants.DriveConstants.kPracRBID); */

      leftVictorEncode = new Encoder(Constants.DriveConstants.kPracLEncode3, Constants.DriveConstants.kPracLEncode4,
          false, EncodingType.k4X); // The external encoder on the practice robot
      rightVictorEncode = new Encoder(Constants.DriveConstants.kPracREncode0, Constants.DriveConstants.kPracREncode1,
          false, EncodingType.k4X);

      m_leftmotors = new SpeedControllerGroup(motorLT, motorLB); // Classifying left side motors
      m_rightmotors = new SpeedControllerGroup(motorRT, motorRB); // Classifying right side motors

      m_rightSide = new DrivetrainPIDSubsystem(m_rightmotors, null, rightVictorEncode);
      m_leftSide = new DrivetrainPIDSubsystem(m_leftmotors, null, leftVictorEncode);

      leftVictorEncode.reset();
      rightVictorEncode.reset();

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

      leftSparkEncode.setPosition(0);
      rightSparkEncode.setPosition(0);

    }

    m_rightSide.enable(); // Enables the PID loop
    m_leftSide.enable(); // Enables the PID loop

  }
  // These are for SmartDash
  public static Encoder getVictorLeft() {
    return leftVictorEncode;
  }

  public static Encoder getVictorRight() {
    return rightVictorEncode;
  }

  public static CANEncoder getSparkLeft() {
    return leftSparkEncode;
  }

  public static CANEncoder getSparkRight() {
    return rightSparkEncode;
  }

  public static void tankDrive(double speedL, double speedR, boolean isTurbo, boolean isSlow)
  {
    tankDrive(speedL, speedR, isTurbo, isSlow, false);
  }

  public static void tankDrive(double varLeft, double varRight, boolean isTurbo, boolean isSlow, boolean shouldSmoothDeceleration) {
    double dTime = Timer.getFPGATimestamp() - timeAtLastTankDrive; // Drive time
    timeAtLastTankDrive = Timer.getFPGATimestamp(); // Timer.getFPGATTimestamp returns the system's clock time in seconds

    if(shouldSmoothDeceleration && Math.abs(varLeft) < Math.abs(leftVector))
      leftVector = JoystUtil.lerp(leftVector, varLeft, dTime*SMOOTH_DECELERATION_COEFF);
    else
      leftVector = varLeft;
    if(shouldSmoothDeceleration && Math.abs(varRight) < Math.abs(rightVector))
      rightVector = JoystUtil.lerp(rightVector, varRight, dTime*SMOOTH_DECELERATION_COEFF);
    else
      rightVector = varRight;

    double speedMulti = 1;
    if(isTurbo)
      speedMulti = 1.5;
    else if(isSlow)
      speedMulti = 0.33;

    m_rightSide.setSetpoint(rightVector * speedMulti);
    m_leftSide.setSetpoint(-leftVector * speedMulti);
  }

  public static void resetEncode(){
    if (leftSparkEncode != null && rightSparkEncode != null) {
      leftSparkEncode.setPosition(0);
      rightSparkEncode.setPosition(0);
    } else if (leftVictorEncode != null && rightVictorEncode != null) {
      leftVictorEncode.reset();
      rightVictorEncode.reset();
    }
  }

  public static double readLeftEncode(){
    return leftSparkEncode.getPosition();
  }

  public static double readRightEncode(){
    return rightSparkEncode.getPosition();
  }

  public static void setPosition(double leftSet, double rightSet) {
    double motorSpeedLeft;
    double motorSpeedRight;

    if (leftSparkEncode != null && rightSparkEncode != null) { // Position set from the right spark max
      motorSpeedLeft = m_PIDleft.calculate(leftSparkEncode.getPosition(), leftSet);
      motorSpeedRight = m_PIDright.calculate(rightSparkEncode.getPosition(), -rightSet);

    } else if (leftVictorEncode != null && rightVictorEncode != null) { // Position set from the left spark max
      motorSpeedLeft = m_PIDleft.calculate(leftVictorEncode.getDistance(), leftSet);
      motorSpeedRight = m_PIDright.calculate(rightVictorEncode.getDistance(), -rightSet);

    } else {
      motorSpeedLeft = 0;
      motorSpeedRight = 0;
    }
    m_leftSide.setSetpoint(motorSpeedLeft);
    m_rightSide.setSetpoint(motorSpeedRight);

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Vector Left: ", leftVector);
    SmartDashboard.putNumber("Vector Right: ", rightVector);

    if (!RobotContainer.isPractice) {
      // Prints speed of encoder
      SmartDashboard.putNumber("VelocityMotorLT: ", ((CANSparkMax) motorLT).getEncoder().getVelocity());
      SmartDashboard.putNumber("VelocityMotorLB: ", ((CANSparkMax) motorLB).getEncoder().getVelocity());
      SmartDashboard.putNumber("VelocityMotorRT: ", ((CANSparkMax) motorRT).getEncoder().getVelocity());
      SmartDashboard.putNumber("VelocityMotorRB: ", ((CANSparkMax) motorRB).getEncoder().getVelocity());

      SmartDashboard.putNumber("Encode Left", ((CANSparkMax) motorLT).getEncoder().getPosition());
      SmartDashboard.putNumber("Encode Right", ((CANSparkMax) motorRT).getEncoder().getPosition());

      // Prints current in amps
      SmartDashboard.putNumber("CurrentMotorLT: ", ((CANSparkMax) motorLT).getOutputCurrent());
      SmartDashboard.putNumber("CurrentMotorLB: ", ((CANSparkMax) motorLB).getOutputCurrent());
      SmartDashboard.putNumber("CurrentMotorRT: ", ((CANSparkMax) motorRT).getOutputCurrent());
      SmartDashboard.putNumber("CurrentMotorRB: ", ((CANSparkMax) motorRB).getOutputCurrent());

    } else {
      SmartDashboard.putNumber("Prac Encode Right: ", rightVictorEncode.getDistance());
      SmartDashboard.putNumber("Prac Encode Left: ", leftVictorEncode.getDistance());
    }

    // This method will be called once per scheduler run

  }

}
