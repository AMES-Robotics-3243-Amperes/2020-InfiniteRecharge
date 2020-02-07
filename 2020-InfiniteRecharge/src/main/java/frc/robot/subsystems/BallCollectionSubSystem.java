/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* Todo: 
      - Determine neccesary class imports
      - Find out what motors and classes are being used
      - PIDs?
*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class BallCollectionSubSystem extends SubsystemBase {
  
  static VictorSPX motorSpin = new VictorSPX(Constants.BallCollectConstants.kSpinID);
  static VictorSPX motorActuate = new VictorSPX(Constants.BallCollectConstants.kActuateID);
  private double deployInitTime;

  public BallCollectionSubSystem() {

  }

  public void setDeployed()
  {
    deployInitTime = Timer.getFPGATimestamp();
    motorActuate.set(ControlMode.Velocity, 0.5);
  }
  
  public void setSpin(boolean spin){ //boolean bc it's a button
    if (spin) {
      motorSpin.set(ControlMode.Velocity, .65); // sets the motor to 65% speed
    }
    else {
      motorSpin.set(ControlMode.Velocity, 0); // if the button is not pressed it sets the motor to 0% speed
    }
  }

  @Override
  public void periodic() {
    if(Timer.getFPGATimestamp() > deployInitTime + 1); // actuator runs for 1 second
  }
}
