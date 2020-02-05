/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;

public class ClimbSubsystem extends SubsystemBase {
  
  private static CANSparkMax climber = new CANSparkMax(8, MotorType.kBrushless);
  private static CANPIDController pidControl;

  public static ADXRS450_Gyro climbSensor = new ADXRS450_Gyro();


  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double min = 0.0;
  double max = 0.0;
  static double angle = 0.0;

  public ClimbSubsystem() {
    pidControl = climber.getPIDController();

    pidControl.setP(kp);
    pidControl.setI(ki);
    pidControl.setD(kd);
    pidControl.setOutputRange(min, max);
  }

  public static void setClimb(boolean value) {
    if (value) {
      pidControl.setReference(angle, ControlType.kPosition);
    }
  }

  public static double getClimb(){
    return climbSensor.getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
