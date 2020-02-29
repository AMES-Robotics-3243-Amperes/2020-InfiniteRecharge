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
  static CANSparkMax dumpShoot;

  static CANEncoder encodeCollect;
  static CANEncoder encodeShoot;

  static CANPIDController pidCollect;
  static CANPIDController pidShoot;

  private static final double ballRotation = 2; // We don't know the correct rotations yet

  // Good enough 2/25/20
  // Collector Constants
  double kp = 0.7;
  double ki = 1.5e-3;
  double kd = 5e-7;
  double min = -0.99;
  double max = 0.99;

  // Still need to tune these constants 2/25/20
  // Shooter Constants
  // Todo:
  /*
  - Lower RPM for new goal (currently -5500) - needs to go into low goal, rather than the high one
  - tune PID constants to correct oscillation
  - 
  */
  double kpShoot = 6e-4;
  double kiShoot = 0;
  double kdShoot = 1e-2;

  static double encodePosition = 0.0;
  static final double encodeVelocity = -5700;  // 5700 is max rpm. Negative to invert motor

  public DumperSubsystem() {
    dumpCollect = new CANSparkMax(Constants.IndexerConstants.kIndexCollectID, MotorType.kBrushless);
    dumpShoot = new CANSparkMax(Constants.IndexerConstants.kIndexShootID, MotorType.kBrushless);
    
    encodeCollect = dumpCollect.getEncoder();
    pidCollect = dumpCollect.getPIDController();

    encodeShoot = dumpShoot.getEncoder();
    pidShoot = dumpShoot.getPIDController();

    pidCollect.setP(kp);
    pidCollect.setI(ki);
    pidCollect.setD(kd);
    pidCollect.setOutputRange(min, max);

    pidShoot.setP(kpShoot);
    pidShoot.setI(kiShoot);
    pidShoot.setD(kdShoot);

  }

  public void setDumpCollectSpeed(boolean shoot, boolean backwards)
  {
    // Backwards not work? Not tested.
    if(shoot && !backwards){
      encodePosition = encodeCollect.getPosition() + ballRotation;
      pidCollect.setReference(encodePosition, ControlType.kPosition);
      System.err.println("### Forwards works ###");
    } else if(backwards && !shoot){
      encodePosition = encodeCollect.getPosition() - ballRotation;
      pidCollect.setReference(encodePosition, ControlType.kPosition);
      System.err.println("### Backwards works ###");
    } else {
      dumpCollect.stopMotor();
    }
    
  }

  public static void setDumpForward(){
    encodePosition = encodeCollect.getPosition() + ballRotation;
    pidCollect.setReference(encodePosition, ControlType.kPosition);
    System.err.println("########## Collect Forward: " + encodePosition + " ##########");
  }

  public static void setDumpBackward(){
    encodePosition = encodeCollect.getPosition() - ballRotation;
    pidCollect.setReference(encodePosition, ControlType.kPosition);
    System.err.println("########## Collect Backward: " + encodePosition + " ##########");
  }

  public static void stopDump(){
    dumpCollect.stopMotor();
  }
  
  // WE STILL NEED TO SET THE PID LOOPS ON THIS.
  // ALSO, WE WOULD CHANGE THE SPD HERE W/ LIMELIGHT CODE
  public static void setDumpHighSpeed(){
    pidShoot.setReference(encodeVelocity, ControlType.kVelocity);
    //dumpShoot.set(-1);
    System.err.println("##### RPM: " + encodeShoot.getVelocity() + " ######");
  }

  public static void setDumpLowSpeed(){
    double lowRPM = -4000;
    pidShoot.setReference(lowRPM, ControlType.kVelocity);
    //dumpShoot.set(-0.55);
    System.err.println("##### RPM: " + encodeShoot.getVelocity() + " ######");
  }

  public static void stopShoot(){
    dumpShoot.stopMotor();
  }

  public void setShootSpeedOnOrOff(boolean shooterOn)
  {
    if(shooterOn){
      //pidShoot.setReference(encodeVelocity, ControlType.kVelocity);
      dumpShoot.set(-1);
    } 
    else{
      pidShoot.setReference(0, ControlType.kVelocity);
    }
  }

  /** Call to set the PID target speed for the shooter. Positive speed = shoot */
  public void setShootSpeed(double rpm)
  {
    pidShoot.setReference( - rpm, ControlType.kVelocity);
  }
  
  public void shootBall(){
    //Make motor dumpShoot spin continously
    //Check for a certain period of time to pass
    //Move dumpCollect dumpCollect to a certain spot
    setShootSpeedOnOrOff(true);
    // 5700 is the free speed limit of the shooter
    if(encodeShoot.getVelocity() >= encodeVelocity - 500 && encodeShoot.getVelocity() <= 5700) 
      //setDumpCollectSpeed(true, false);
      setDumpForward();
    // TODO: shooter needs to turn off when this method isn't called continuously
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Collect Encoder", encodeCollect.getPosition());
    SmartDashboard.putNumber("Shooter RPM", encodeShoot.getVelocity());
  }

}