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

  static CANSparkMax dumpCollect;
  static CANSparkMax dumpShoot;

  static CANEncoder encodeCollect;
  static CANEncoder encodeShoot;

  static CANPIDController pidCollect;
  static CANPIDController pidShoot;

  //! Maybe change this???
  private static final double ballRotation = 10; // We don't know the correct rotations yet
  private double rotateStop = 0.0;
  private final double ROTATE_LIMIT = 200.0; // Is this in RPM?

  // Collector Constants
  double kp = 0.7;
  double ki = 1.5e-3;
  double kd = 5e-7;
  double min = -0.99;
  double max = 0.99;

  // Shooter Constants
  // Todo:
  /*
   * - Lower RPM for new goal (currently -5500) - needs to go into low goal,
   * rather than the high one - tune PID constants to correct oscillation -
   ! Run the dumper with velocity instead of position
   */
  static double kpShoot = 6e-4; // PID constan
  double kiShoot = 0;
  double kdShoot = 20;

  static double encodePosition = 0.0;
  static final double autoPosition = 200;
  static final double encodeVelocity = -6500; // 5700 is max rpm. Negative to invert motor (Free Speed) RPM
  static final double encodeMedVelocity = -5000;

  //Dumper speeds
  static final double dumperSpeed = 10;
  static final double dumperSpeedNoShoot = 11;

  final double lowRPM = -3500; //lowest speed

  public DumperSubsystem() {
    dumpCollect = new CANSparkMax(Constants.IndexerConstants.kIndexCollectID, MotorType.kBrushless);
    dumpShoot = new CANSparkMax(Constants.IndexerConstants.kIndexShootID, MotorType.kBrushless);
    dumpShoot.setSmartCurrentLimit(37);

    encodeCollect = dumpCollect.getEncoder();
    pidCollect = dumpCollect.getPIDController();

    encodeShoot = dumpShoot.getEncoder();
    pidShoot = dumpShoot.getPIDController();

    pidCollect.setP(kp);
    pidCollect.setI(ki);
    pidCollect.setD(kd);
    pidCollect.setOutputRange(min, max); 
    // the .setP() is set in the two different High and Low shooting methods
    pidShoot.setI(kiShoot);
    pidShoot.setD(kdShoot);
    pidShoot.setOutputRange(-1, 1);

  }

  //! New code for dumper here: 
  //* Note: not tested yet
  //? Change domp to dump when wanting to test!

  public void setDompCollectSpeed(boolean shoot, boolean backwards) {
    if (shoot && !backwards) {
      dumpCollect.set(dumperSpeed);
    } else if (backwards && !shoot) {
      dumpCollect.set(dumperSpeedNoShoot);
    } else {
      dumpCollect.stopMotor();
    }
  }

  public void setDompForward(boolean auto) {
    if (!auto) {
      dumpCollect.set(dumperSpeed);
    } else {
      pidCollect.setReference(autoPosition, ControlType.kPosition);
    }
  }

  public void setDompBackward() {
    dumpCollect.set(-dumperSpeed);
  }

  //! End of test code

  public void setDumpCollectSpeed(boolean shoot, boolean backwards) {
    if (shoot && !backwards) {
      encodePosition = encodeCollect.getPosition() + ballRotation;
      pidCollect.setReference(encodePosition, ControlType.kPosition);
    } else if (backwards && !shoot) {
      encodePosition = encodeCollect.getPosition() - ballRotation;
      pidCollect.setReference(encodePosition, ControlType.kPosition);
    } else {
      dumpCollect.stopMotor();
    }

  }

  public void setDumpForward(boolean auto) {
    if( ! auto){
      encodePosition = encodeCollect.getPosition() + ballRotation;
      pidCollect.setReference(encodePosition, ControlType.kPosition);
    } else {
      pidCollect.setReference(autoPosition, ControlType.kPosition);
    }

  }

  public void setDumpBackward() {
    encodePosition = encodeCollect.getPosition() - ballRotation;
    pidCollect.setReference(encodePosition, ControlType.kPosition);
  }

  public void resetIndexer(){
    encodeCollect.setPosition(0);
  }

  public double readIndexer(){
    return encodeCollect.getPosition();
  }

  public void stopDump() {
    dumpCollect.stopMotor();
  }

  public static void setDumpHighSpeed() { // sets the speed to a high 6500 rpm
    pidShoot.setP(kpShoot);
    pidShoot.setReference(encodeVelocity, ControlType.kVelocity);
  }

  public void setDumpLowSpeed() { // sets the speed to a low 1500 rpm
    pidShoot.setP(kpShoot / 10);
    pidShoot.setReference(lowRPM, ControlType.kVelocity);
  }
  public void setDumpMedSpeed(){ // provides a moderate speed of 3500 rpm
    pidShoot.setP(kpShoot / 5);
    pidShoot.setReference(encodeMedVelocity, ControlType.kVelocity);
  }

  public static void stopShoot() {
    dumpShoot.stopMotor();
  }

  public void setShootSpeedOnOrOff(boolean shooterOn) {
    if (shooterOn) {
      // pidShoot.setReference(encodeVelocity, ControlType.kVelocity);
      dumpShoot.set(-1);
    } else {
      pidShoot.setReference(0, ControlType.kVelocity);
    }
  }

  /** Call to set the PID target speed for the shooter. Positive speed = shoot */
  public void setShootSpeed(double rpm) {
    pidShoot.setReference(-rpm, ControlType.kVelocity);

  }

  public void shootBall() { // For autonomous
    // Make motor dumpShoot spin continously
    // Check for a certain period of time to pass
    // Move dumpCollect dumpCollect to a certain spot

    rotateStop = encodeCollect.getPosition();

    setDumpHighSpeed();

    // If the shooter rpm is within 4800 to 5700 and the indexer hasn't pushed all of the balls through the shooter
    if (encodeShoot.getVelocity() <= encodeVelocity + 900 && encodeShoot.getVelocity() >= encodeVelocity
        && rotateStop <= encodeCollect.getPosition() + ROTATE_LIMIT){
      setDumpForward(true);
    } // Got rid of the else{} where it stops the motor and resets everything again
    
    // TODO: shooter needs to turn off when this method isn't called continuously
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Collect Encoder", encodeCollect.getPosition());
    SmartDashboard.putNumber("Shooter RPM", encodeShoot.getVelocity());
  }

}