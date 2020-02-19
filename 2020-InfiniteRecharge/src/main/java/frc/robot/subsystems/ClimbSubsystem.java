/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

import frc.robot.Constants;
import frc.robot.util.PIDMotor;

/* Two NEOs are going to push both elevator-like beams up to grab the bar.
   Both elevator ends have limit switches to stop the elevator from moving too far.
   One NEO is on the hook to tilt the robot to the correct angle.
 */

public class ClimbSubsystem extends SubsystemBase {
  
  private static CANSparkMax climberADJ = new CANSparkMax(Constants.ClimbingConstant.kClimbAdjID, MotorType.kBrushless);
  private static CANSparkMax climberR = new CANSparkMax(Constants.ClimbingConstant.kClimbRID, MotorType.kBrushless);
  private static CANSparkMax climberL = new CANSparkMax(Constants.ClimbingConstant.kClimbLID, MotorType.kBrushless);
  private static PIDMotor climberADJPID = new PIDMotor(climberADJ);
  private static CANPIDController pidControlRight;
  private static CANPIDController pidControlLeft;
  private static CANEncoder encodeLeft;
  private static CANEncoder encodeRight;

  // NOT YET TUNED TO THE ROBOT! 2/5/20
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double min = 0.0;
  double max = 0.99;

  static double leftPosition = 0.0;
  static double rightPosition = 0.0;

  public ClimbSubsystem() {
    pidControlRight = climberR.getPIDController();
    pidControlLeft = climberL.getPIDController();

    pidControlRight.setP(kp);
    pidControlRight.setI(ki);
    pidControlRight.setD(kd);
    pidControlRight.setOutputRange(min, max);

    pidControlLeft.setP(kp);
    pidControlLeft.setI(ki);
    pidControlLeft.setD(kd);
    pidControlLeft.setOutputRange(min, max);

    encodeRight = climberR.getEncoder();
    encodeLeft = climberL.getEncoder();

    climberADJ.setSmartCurrentLimit(39);
    climberR.setSmartCurrentLimit(39);
    climberL.setSmartCurrentLimit(39);
  }

  public static void setClimb(double leftSpd, double rightSpd, boolean actuateUp) {

    double maxRotations = 0.0;  // DON'T KNOW WHAT THE MAX ROTATIONS IS YET 2/16/20

    // One of these MUST BE NEGATIVE, so we have to test it 2/16/20
    leftPosition = encodeLeft.getPosition() + leftSpd;
    rightPosition = encodeRight.getPosition() + rightSpd;

    // Moves the climbing system up
    if(actuateUp){
      climberADJPID.setPIDPosition(5);
    } else if(!actuateUp){
      climberADJPID.setPIDPosition(0);
    }

    // Left climber side
    if(encodeLeft.getPosition() <= maxRotations){ // Move position if not hit max position
      pidControlLeft.setReference(leftPosition, ControlType.kPosition);
    } else if(encodeLeft.getPosition() > maxRotations){
      if(leftPosition > encodeLeft.getPosition()){
        // if we force motor to go past max position, then stop motor
        climberL.stopMotor();
      } else if(leftPosition < encodeLeft.getPosition()){
        // if we want motor to lower position, then let it move
        pidControlLeft.setReference(leftPosition, ControlType.kPosition);
      }
    }

    // Right climber side
    if(encodeRight.getPosition() <= maxRotations){  // Move position if not hit max position
      pidControlRight.setReference(rightPosition, ControlType.kPosition);
    } else if(encodeRight.getPosition() > maxRotations){
      if(rightPosition > encodeRight.getPosition()){
        // if we force motor to go past max position, then stop motor
        climberR.stopMotor();
      } else if(rightPosition < encodeRight.getPosition()){
        // if we want motor to lower position, then let it move
        pidControlRight.setReference(rightPosition, ControlType.kPosition);
      }
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Done by Alejandro. Not sure if its right or needed
    SmartDashboard.getNumber("ClimberADJ Current: ", climberADJ.getOutputCurrent()); // Prints current in amps
    SmartDashboard.getNumber("ClimberR Current: ", climberR.getOutputCurrent());
    SmartDashboard.getNumber("ClimberL Current: ", climberL.getOutputCurrent());

    SmartDashboard.getNumber("ClimberADJ Volt: ", climberADJ.getBusVoltage()); // Prints the voltage going into the motor controller
    SmartDashboard.getNumber("ClimberR Volt: ", climberR.getBusVoltage());
    SmartDashboard.getNumber("ClimberL Volt: ", climberL.getBusVoltage());
  }
}
