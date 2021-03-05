// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Setting up code for angle adustment of the shooter
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ActuatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// ^ Delectible libraries ^

public class ActuatorSubsystem extends SubsystemBase {
  /** Creates a new ActuatorSubsystem. */
  private static Servo actuatorServo;
  //private static servoBallAdj instance = null;
  private static final double MIN_ANGLE_DEG = 0.0;
  private static final double MAX_ANGLE_DEG = 100;
  private static final boolean inverted = false;
  public ActuatorSubsystem() { // Le actuator constructor
    actuatorServo = new Servo(frc.robot.Constants.ActuatorConstants.actuatorConnectorOne); 
    setActAngle(MIN_ANGLE_DEG);
  }


  public void setActAngle(double angle){ //sets the angle for the actuator
    setAngle(angle, actuatorServo);
  }


  private static void setAngle(double angle, Servo servo){ //plugs in values from setActAngle
    servo.setAngle(angle);

  }

  public double getActuatorAngle(){ //returns values of actuator angle and angle
    return getAngle(actuatorServo, MAX_ANGLE_DEG);
  }

  public double getAngle(Servo servo, double maxAngle){ //Retrieves servo angle for actuator
    double angle = servo.getAngle();
    return angle; 
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
