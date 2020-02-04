/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

  static CANSparkMax shoot = new CANSparkMax(0, MotorType.kBrushless);
  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {

  }

  public static void setShoot(boolean value){ //Tune the speed as necessary
    if(value){
      shoot.set(0.70);  // Set shoot motor to 70% spd
    } else if(!value){
      shoot.set(0.0); // Set shoot motor to 0% spd
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
