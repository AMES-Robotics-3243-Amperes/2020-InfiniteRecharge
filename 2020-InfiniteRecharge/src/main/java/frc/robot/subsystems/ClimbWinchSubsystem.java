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
import edu.wpi.first.wpilibj.controller.PIDController;
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

public class ClimbWinchSubsystem extends SubsystemBase {
  // MOTOR DIRECTIONS
  // 1 Extends Left
  // -1 Extends Right
  // -1 Deploys Winch
  public CANSparkMax climberWinch = new CANSparkMax(Constants.ClimbingConstant.kClimbAdjID, MotorType.kBrushless);
  public PIDMotor climberWinchPID = new PIDMotor(climberWinch);

  private DigitalInput limitSwitchWinchBottom = new DigitalInput(Constants.ClimbingConstant.kBottomWinchLimitID);
  private DigitalInput limitSwitchWinchTop = new DigitalInput(Constants.ClimbingConstant.kTopWinchLimitID);

  private final double WINCH_DEPLOYED_ROTS = -9 * 100; // 100:1 gearbox ratio  CHANGED FROM 3 TO 5 ROTATIONS
  private final double WINCH_TARGET_MARGIN_ROTS = 0.25 * 100;

  // NOT YET TUNED TO THE ROBOT! 2/5/20
  double kp = 0.65;
  double ki = 0.0;
  double kd = 1e-8;
  double min = -0.25;
  double max = 0.25;

  public boolean areEncodersReset = false;

  public ClimbWinchSubsystem() {

    climberWinchPID.setOutputRange(-1, 1);

    climberWinch.setSmartCurrentLimit(39);
  }

  /** Call periodically to deploy/retract the winch */
  public void winchPeriodic(boolean shouldDeploy) {
    climberWinch.set(shouldDeploy ?-1.0 : 1.0);
    enforceMotorRangeSafeguards();
  }

  /** Ccall once to control the winch with PID */
  public void setWinchIsDeployed(boolean shouldDeploy) {
    setWinchTarget(shouldDeploy ?WINCH_DEPLOYED_ROTS :0);
  }

  public void setWinchTarget(double rotations) {
    climberWinchPID.setPIDPosition(rotations);
    enforceMotorRangeSafeguards(); // Don't wait until periodic() to run this; If something's wrong, we need to stop immediately!
  }

  public double getWinchPosition() {
    return climberWinchPID.encoder.getPosition();
  }

  public void stopAllMotors() {
    climberWinch.stopMotor();
  }
  public void stopWinch()
  { climberWinch.stopMotor(); }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Winch rots", climberWinchPID.encoder.getPosition());

    enforceMotorRangeSafeguards();

    //SmartDashboard.putNumber("ClimberADJ Current: ", climberWinch.getOutputCurrent()); // Prints current in amps

    SmartDashboard.putBoolean("isWinchDeployed()", isWinchDeployed());
    SmartDashboard.putBoolean("isWinchRetracted()", isWinchRetracted());
    SmartDashboard.putBoolean("limitSwitch WinchBottom", limitSwitchWinchBottom.get());
    SmartDashboard.putBoolean("limitSwitch WinchTop", limitSwitchWinchTop.get());
  }

  private void enforceMotorRangeSafeguards()
  {
    
    /*if((climberWinchPID.encoder.getVelocity() < -1 && isWinchRetracted())
      || (climberWinchPID.encoder.getVelocity() > 1 && isWinchDeployed())){
      System.err.println(" #### THE WINCH DOES STOP ##### ");
      climberWinch.stopMotor();
      }*/

      // Limit switch-only safeguards
    if(climberWinchPID.encoder.getVelocity() < -1 && limitSwitchWinchTop.get())
      climberWinch.stopMotor();
    if(climberWinchPID.encoder.getVelocity() > 1 && limitSwitchWinchBottom.get())
      climberWinch.stopMotor();
  }

  public boolean isWinchDeployed()
  {
    return climberWinchPID.encoder.getPosition() <= WINCH_DEPLOYED_ROTS + WINCH_TARGET_MARGIN_ROTS
      || limitSwitchWinchTop.get();
  }
  public boolean isWinchRetracted()
  {
    return (climberWinchPID.encoder.getPosition() >= - WINCH_TARGET_MARGIN_ROTS && areEncodersReset) // Don't let the encoders stop motors while resetting is happening
      || limitSwitchWinchBottom.get();
  }

  public boolean isWinchHalfRetracted()
  {
    return (climberWinchPID.encoder.getPosition() >= - WINCH_DEPLOYED_ROTS/2.0 && areEncodersReset) || isWinchRetracted(); // Don't let the encoders stop motors while resetting is happening
  }

  public boolean resetRetractWinch()
  {
    boolean lSwitch = limitSwitchWinchBottom.get();
    climberWinch.set(lSwitch ?0 :1);
    if(lSwitch)
        climberWinchPID.encoder.setPosition(0);
    return lSwitch;
  }
}
