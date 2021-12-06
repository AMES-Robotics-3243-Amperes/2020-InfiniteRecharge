/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.util.PIDMotor;

/* Two NEOs are going to push both elevator-like beams up to grab the bar.
   Both elevator ends have limit switches to stop the elevator from moving too far.
   One NEO is on the hook to tilt the robot to the correct angle.
 */

public class ClimbArmsSubsystem extends SubsystemBase {
  // MOTOR DIRECTIONS
  // 1 Extends Left
  // -1 Extends Right
  // -1 Deploys Winch
  public CANSparkMax climberR = new CANSparkMax(Constants.ClimbingConstant.kClimbRID, MotorType.kBrushless);
  public CANSparkMax climberL = new CANSparkMax(Constants.ClimbingConstant.kClimbLID, MotorType.kBrushless);
  private PIDMotor pidControlRight = new PIDMotor(climberR);
  private PIDMotor pidControlLeft = new PIDMotor(climberL);

  private DigitalInput limitSwitchLeft = new DigitalInput(Constants.ClimbingConstant.kLeftLimitID);
  private DigitalInput limitSwitchRight = new DigitalInput(Constants.ClimbingConstant.kRightLimitID);

  private final double ARM_EXTENDED_ROTS = -3.75 * 64; // 64:1 gearbox ratio
  private final double ARM_CONTROL_PANEL_POSITION_ROTS = 5;
  private final double ARM_TARGET_MARGIN_ROTS = 0.1 * 64; // Changed from 0.25 to 0.1

  double kp = 0.65;
  double ki = 0.0;
  double kd = 1e-8;
  double min = -0.25;
  double max = 0.25;

  static double leftPosition = 0.0;
  static double rightPosition = 0.0;

  public boolean areEncodersReset = false;

  public ClimbArmsSubsystem() {

    pidControlLeft.setP(kp);
    pidControlLeft.setI(ki);
    pidControlLeft.setD(kd);

    pidControlRight.setP(kp);
    pidControlRight.setI(ki);
    pidControlRight.setD(kd);

    climberR.setSmartCurrentLimit(39);
    climberL.setSmartCurrentLimit(39);
  }

  public void extendArmsForClimbing() {
    setLeftExtendTarget( - ARM_EXTENDED_ROTS);
    setRightExtendTarget(ARM_EXTENDED_ROTS);
  }

  public void retractArms() {
    setLeftExtendTarget(0);
    setRightExtendTarget(0);
  }
  
  public void extendArmControlPanelMechanism()
  {
    setRightExtendTarget(ARM_CONTROL_PANEL_POSITION_ROTS);
  }

  public void setLeftExtendTarget(double rotations)
  {
    pidControlLeft.setPIDPosition(rotations);
    enforceMotorRangeSafeguards(); // Don't wait until periodic() to run this; If something's wrong, we need to stop immediately!
  }
  public void setRightExtendTarget(double rotations)
  {
    pidControlRight.setPIDPosition(rotations);
    enforceMotorRangeSafeguards(); // Don't wait until periodic() to run this; If something's wrong, we need to stop immediately!
  }

  public void setLeftVelocity(double velocity){
    
    if((!isLeftarmRetracted() && velocity > 0) || (!isLeftArmExtended() && velocity < 0)){
      pidControlLeft.setPIDPosition(pidControlLeft.encoder.getPosition() - 2.25 * velocity);
    }

  }
  public void setRightVelocity(double velocity){

    if((!isRightArmRetracted() && velocity > 0) || (!isRightArmExtended() && velocity < 0)){
      pidControlRight.setPIDPosition(pidControlRight.encoder.getPosition() + 2.25 * velocity);
    }

  }
  
  public double getLeftArmPosition()
  {
    return pidControlLeft.encoder.getPosition();
  }
  public double getRightArmPosition()
  {
    return pidControlRight.encoder.getPosition();
  }

  public void stopAllMotors() {
    climberR.stopMotor();
    climberL.stopMotor();
  }
  public void stopArmRight()
  { climberR.stopMotor(); }
  public void stopArmLeft()
  { climberL.stopMotor(); }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("climbL rots", pidControlLeft.encoder.getPosition());
    SmartDashboard.putNumber("climbR rots", pidControlRight.encoder.getPosition());
    SmartDashboard.putBoolean("isClimberArmExtended", isClimberArmExtended());

    enforceMotorRangeSafeguards();

    SmartDashboard.putBoolean("isLeftArmExtended()", isLeftArmExtended());
    SmartDashboard.putBoolean("isLeftArmRetracted()", isLeftarmRetracted());
    SmartDashboard.putBoolean("isRightArmExtended()", isRightArmExtended());
    SmartDashboard.putBoolean("isRightArmRetracted()", isRightArmRetracted());
    SmartDashboard.putBoolean("limitSwitch Right", limitSwitchRight.get());
    SmartDashboard.putBoolean("limitSwitch Left", limitSwitchLeft.get());
  }

  private void enforceMotorRangeSafeguards()
  {
    // Arm extension safeguard; If near limit and moving toward limit, stop the motor.
    if((pidControlRight.encoder.getVelocity()>1 && isRightArmRetracted()) // Velocity in RPM
      || (pidControlRight.encoder.getVelocity()<-1 && isRightArmExtended())){
        System.err.println("#### THE RIGHT CLIMBER DOES STOP ####");
        climberR.stopMotor();
      }

    if((pidControlLeft.encoder.getVelocity()<-1 && isLeftarmRetracted())
      || (pidControlLeft.encoder.getVelocity()>1 && isLeftArmExtended())){
      System.err.println("#### THE LEFT CLIMBER DOES STOP ####");
      climberL.stopMotor();
      }
  }

  public boolean isControlPanelLifted()
  {
    return Math.abs(pidControlRight.encoder.getPosition() - ARM_CONTROL_PANEL_POSITION_ROTS) <= ARM_TARGET_MARGIN_ROTS;
  }

  public boolean isClimberArmExtended() {
    return isLeftArmExtended() && isRightArmExtended(); // Checks if arm is retracted when method is for extended
  }
  public boolean isLeftArmExtended()
  {
    return pidControlLeft.encoder.getPosition() >= - ARM_EXTENDED_ROTS - ARM_TARGET_MARGIN_ROTS;
  }
  public boolean isRightArmExtended()
  {
    return pidControlRight.encoder.getPosition() <= ARM_EXTENDED_ROTS + ARM_TARGET_MARGIN_ROTS;
  }

  public boolean isClimberArmRetracted() {
    return isRightArmRetracted() && isLeftarmRetracted();
  }
  public boolean isLeftarmRetracted()
  {
    return (pidControlLeft.encoder.getPosition() <= ARM_TARGET_MARGIN_ROTS && areEncodersReset)
      || limitSwitchLeft.get();
  }
  public boolean isRightArmRetracted()
  {
    return (pidControlRight.encoder.getPosition() >= -ARM_TARGET_MARGIN_ROTS && areEncodersReset)
      || limitSwitchRight.get();
  }

  public boolean resetRetractLeft()
  {
    boolean lSwitch = limitSwitchLeft.get();
    climberL.set(lSwitch ?0 :-0.5);
    if(lSwitch)
        pidControlLeft.encoder.setPosition(0);
    return lSwitch;
  }
  public boolean resetRetractRight()
  {
    boolean lSwitch = limitSwitchRight.get();
    climberR.set(lSwitch ?0 :0.5);
    if(lSwitch)
        pidControlRight.encoder.setPosition(0);
    return lSwitch;
  }
}
