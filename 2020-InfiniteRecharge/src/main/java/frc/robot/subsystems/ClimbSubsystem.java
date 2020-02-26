/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

import frc.robot.Constants;
import frc.robot.util.PIDMotor;

/* Two NEOs are going to push both elevator-like beams up to grab the bar.
   Both elevator ends have limit switches to stop the elevator from moving too far.
   One NEO is on the hook to tilt the robot to the correct angle.
 */

public class ClimbSubsystem extends SubsystemBase {
  // MOTOR DIRECTIONS
  // -1 Extends Left
  // 1 Extends Right
  // -1 Deploys Winch
  private CANSparkMax climberWinch = new CANSparkMax(Constants.ClimbingConstant.kClimbAdjID, MotorType.kBrushless);
  private CANSparkMax climberR = new CANSparkMax(Constants.ClimbingConstant.kClimbRID, MotorType.kBrushless);
  private CANSparkMax climberL = new CANSparkMax(Constants.ClimbingConstant.kClimbLID, MotorType.kBrushless);
  private PIDMotor climberWinchPID = new PIDMotor(climberWinch);
  private PIDMotor pidControlRight = new PIDMotor(climberR);
  private PIDMotor pidControlLeft = new PIDMotor(climberL);
  private CANEncoder encodeLeft;
  private CANEncoder encodeRight;
  private CANEncoder encodeWinch;

  private DigitalInput limitSwitchLeft = new DigitalInput(Constants.ClimbingConstant.kLeftLimitID);
  private DigitalInput limitSwitchRight = new DigitalInput(Constants.ClimbingConstant.kRightLimitID);
  private DigitalInput limitWinchBottom = new DigitalInput(Constants.ClimbingConstant.kBottomWinchLimitID);
  private DigitalInput limitWinchTop = new DigitalInput(Constants.ClimbingConstant.kTopWinchLimitID);

  Servo stopClimb = new Servo(Constants.ClimbingConstant.kServoID);

  private final double ARM_CURRENT_LIMIT = 40;

  private final double ARM_EXTENDED_ROTS = 3.75 * 64; // 64:1 gearbox ratio
  private final double ARM_CONTROL_PANEL_POSITION_ROTS = 5;
  private final double ARM_TARGET_MARGIN_ROTS = 0.25 * 64;
  private final double WINCH_DEPLOYED_ROTS = -5 * 100; // 100:1 gearbox ratio CHANGED FROM 3 TO 5 ROTATIONS
  private final double WINCH_TARGET_MARGIN_ROTS = 0.25 * 100;

  // NOT YET TUNED TO THE ROBOT! 2/5/20
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double min = -0.25;
  double max = 0.25;

  static double leftPosition = 0.0;
  static double rightPosition = 0.0;

  public ClimbSubsystem() {
    pidControlRight.setP(kp);
    pidControlRight.setI(ki);
    pidControlRight.setD(kd);
    pidControlRight.setOutputRange(min, max);

    pidControlLeft.setP(kp);
    pidControlLeft.setI(ki);
    pidControlLeft.setD(kd);
    pidControlLeft.setOutputRange(min, max);

    climberWinchPID.setOutputRange(-1, 1);

    encodeRight = climberR.getEncoder();
    encodeLeft = climberL.getEncoder();
    encodeWinch = climberWinch.getEncoder();

    climberWinch.getEncoder().setPosition(0);
    climberR.getEncoder().setPosition(0);
    climberL.getEncoder().setPosition(0);

    climberWinch.setSmartCurrentLimit(39);
    climberR.setSmartCurrentLimit(39);
    climberL.setSmartCurrentLimit(39);
  }

  public boolean setExtendWinch() {

    if (!limitWinchTop.get()) {
      climberWinch.stopMotor();
      return true;

    } else {

      if (encodeWinch.getPosition() < WINCH_DEPLOYED_ROTS + WINCH_TARGET_MARGIN_ROTS) {
        climberWinch.set(-0.25);
      } else {
        climberWinch.set(-0.55);
      }

      return false;
    }

  }

  public boolean setRetractWinch() {

    if (limitWinchBottom.get()) {
      climberWinch.stopMotor();
      return true;

    } else {

      if (encodeWinch.getPosition() < -WINCH_TARGET_MARGIN_ROTS) {
        climberWinch.set(0.25);
      } else {
        climberWinch.set(0.55);
      }

      return false;
    }
  }

  public void setWinchIsDeployed(boolean shouldDeploy) {
    // Moves the climbing system up
    climberWinchPID.setPIDPosition(shouldDeploy ? WINCH_DEPLOYED_ROTS : 0);
  }

  public void setWinchTarget(double rotations) {
    climberWinchPID.setPIDPosition(rotations);
    enforceMotorRangeSafeguards(); // Don't wait until periodic() to run this; If something's wrong, we need to
                                   // stop immediately!
  }

  public double getWinchPosition() {
    return climberWinchPID.encoder.getPosition();
  }

  public void setLatchServoOpen(boolean isOpen) {
    if (isOpen) {
      stopClimb.setAngle(Constants.ClimbingConstant.kMoveServo);
    } else {
      stopClimb.setAngle(Constants.ClimbingConstant.kStopServo);
    }
  }

  public void extendArmsForClimbing() {
    // Left side
    if (-encodeLeft.getPosition() > ARM_EXTENDED_ROTS || climberL.getOutputCurrent() > ARM_CURRENT_LIMIT) {
      climberL.stopMotor();
    } else if (-encodeLeft.getPosition() > ARM_EXTENDED_ROTS - ARM_TARGET_MARGIN_ROTS) {
      climberL.set(-0.25);
    } else {
      climberL.set(-0.5);
    }

    // Right side
    if (encodeRight.getPosition() > ARM_EXTENDED_ROTS || climberR.getOutputCurrent() > ARM_CURRENT_LIMIT) {
      climberR.stopMotor();
    } else if (encodeRight.getPosition() > ARM_EXTENDED_ROTS - ARM_TARGET_MARGIN_ROTS) {
      climberR.set(0.25);
    } else {
      climberR.set(0.5);
    }

  }

  public void retractArms() {

    if (-encodeLeft.getPosition() < ARM_TARGET_MARGIN_ROTS || limitSwitchLeft.get()
        || climberL.getOutputCurrent() > ARM_CURRENT_LIMIT) {
      climberL.stopMotor();
    } else {
      climberL.set(-0.5);
    }

    if (encodeRight.getPosition() < ARM_TARGET_MARGIN_ROTS || limitSwitchRight.get()
        || climberR.getOutputCurrent() > ARM_CURRENT_LIMIT) {
      climberR.stopMotor();
    } else {
      climberR.set(0.5);
    }

  }

  public void extendArmControlPanelMechanism() {
    setRightExtendTarget(ARM_CONTROL_PANEL_POSITION_ROTS);
  }

  public void setLeftExtendTarget(double rotations) {
    pidControlLeft.setPIDPosition(rotations);
    System.err.println("#####   LEFT side EXTENDS   #####");
    enforceMotorRangeSafeguards(); // Don't wait until periodic() to run this; If something's wrong, we need to
                                   // stop immediately!
  }

  public void setRightExtendTarget(double rotations) {
    pidControlRight.setPIDPosition(rotations);
    System.err.println("######   RIGHT side EXTENDS   #####");
    enforceMotorRangeSafeguards(); // Don't wait until periodic() to run this; If something's wrong, we need to
                                   // stop immediately!
  }

  public double getLeftArmPosition() {
    return pidControlLeft.encoder.getPosition();
  }

  public double getRightArmPosition() {
    return pidControlRight.encoder.getPosition();
  }

  public void stopAllMotors() {
    climberR.stopMotor();
    climberL.stopMotor();
    climberWinch.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Winch rots", climberWinchPID.encoder.getPosition());

    enforceMotorRangeSafeguards();

    SmartDashboard.putNumber("ClimberADJ Current: ", climberWinch.getOutputCurrent()); // Prints current in amps
    SmartDashboard.putNumber("ClimberR Current: ", climberR.getOutputCurrent());
    SmartDashboard.putNumber("ClimberL Current: ", climberL.getOutputCurrent());

    // Done by Alejandro. Not sure if its right or needed
    SmartDashboard.putNumber("Left extend?", encodeRight.getPosition());
    SmartDashboard.putNumber("Right extend?", encodeLeft.getPosition());

    SmartDashboard.putBoolean("isWinchDeployed()", isWinchDeployed());
    SmartDashboard.putBoolean("isWinchRetracted()", isWinchRetracted());

    //SmartDashboard.putBoolean("Left Limit", limitSwitchLeft.get());
    //SmartDashboard.putBoolean("Right Limit", limitSwitchRight.get());
  }

  private void enforceMotorRangeSafeguards() {

    // Arm extension safeguard; If near limit and moving toward limit, stop the
    // motor.
    if (((pidControlRight.encoder.getVelocity() < 0
        && Math.abs(pidControlRight.encoder.getPosition() - 0) < ARM_TARGET_MARGIN_ROTS))
        || (pidControlRight.encoder.getVelocity() > 0
            && Math.abs(pidControlRight.encoder.getPosition() - ARM_EXTENDED_ROTS) < ARM_TARGET_MARGIN_ROTS)) {
      climberR.stopMotor();
    }

    if ((pidControlLeft.encoder.getVelocity() > 0
        && Math.abs(pidControlLeft.encoder.getPosition() - 0) < ARM_TARGET_MARGIN_ROTS)
        || (pidControlLeft.encoder.getVelocity() < 0
            && Math.abs(pidControlLeft.encoder.getPosition() - -ARM_EXTENDED_ROTS) < ARM_TARGET_MARGIN_ROTS)) {
      climberL.stopMotor();
    }

  }

  public boolean isWinchDeployed() {
    return !limitWinchTop.get(); // Once it hits the switch, the winch has deployed
    // return climberWinchPID.encoder.getPosition() <= WINCH_DEPLOYED_ROTS +
    // WINCH_TARGET_MARGIN_ROTS;
  }

  public boolean isWinchRetracted() {
    return limitWinchBottom.get(); // Once it hits the switch, the winch has retracted
    // return climberWinchPID.encoder.getPosition() >= - WINCH_TARGET_MARGIN_ROTS;
  }

  public boolean isControlPanelLifted() {
    return Math.abs(pidControlRight.encoder.getPosition() - ARM_CONTROL_PANEL_POSITION_ROTS) <= ARM_TARGET_MARGIN_ROTS;
  }

  public boolean isClimberArmExtended() {
    return isLeftarmRetracted() && isRightArmExtended();
  }

  public boolean isLeftArmExtended() {
    return pidControlLeft.encoder.getPosition() <= ARM_EXTENDED_ROTS + ARM_TARGET_MARGIN_ROTS;
  }

  public boolean isRightArmExtended() {
    return pidControlRight.encoder.getPosition() >= ARM_EXTENDED_ROTS - ARM_TARGET_MARGIN_ROTS;
  }

  public boolean isClimberArmRetracted() {
    return isRightArmRetracted() && isLeftarmRetracted();
  }

  public boolean isLeftarmRetracted() {
    return pidControlLeft.encoder.getPosition() >= -ARM_TARGET_MARGIN_ROTS || limitSwitchLeft.get();
  }

  public boolean isRightArmRetracted() {
    return pidControlRight.encoder.getPosition() <= ARM_TARGET_MARGIN_ROTS || limitSwitchRight.get();
  }
}
