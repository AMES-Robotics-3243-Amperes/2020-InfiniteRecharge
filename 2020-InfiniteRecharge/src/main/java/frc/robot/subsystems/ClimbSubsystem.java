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
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

/* Two NEOs are going to push both elevator-like beams up to grab the bar.
   Both elevator ends have limit switches to stop the elevator from moving too far.
   One NEO is on the hook to tilt the robot to the correct angle.
 */

public class ClimbSubsystem extends SubsystemBase {
  
  private static CANSparkMax climberADJ = new CANSparkMax(Constants.ClimbingConstant.kClimbAdjID, MotorType.kBrushless);
  private static CANSparkMax climberR = new CANSparkMax(Constants.ClimbingConstant.kClimbRID, MotorType.kBrushless);
  private static CANSparkMax climberL = new CANSparkMax(Constants.ClimbingConstant.kClimbLID, MotorType.kBrushless);
  private static CANPIDController pidControl;

  private static DigitalInput limitTop = new DigitalInput(Constants.ClimbingConstant.kLimitTopID);
  private static DigitalInput limitBottom = new DigitalInput(Constants.ClimbingConstant.kLimitBottomID);

  // NOT YET TUNED TO THE ROBOT! 2/5/20
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double min = 0.0;
  double max = 0.0;
  static double angle = 0.0;

  public ClimbSubsystem() {
    pidControl = climberADJ.getPIDController();

    pidControl.setP(kp);
    pidControl.setI(ki);
    pidControl.setD(kd);
    pidControl.setOutputRange(min, max);

    climberR.setSmartCurrentLimit(40);
    climberL.setSmartCurrentLimit(40);
    climberADJ.setSmartCurrentLimit(40);
  }

  public static void setClimb(boolean value, boolean adjust) {

    if (value && !adjust && !limitTop.get() && !limitBottom.get()) {
      climberR.set(0.65);
      climberL.set(0.65);
    } else if(adjust && !value && !limitTop.get() && !limitBottom.get()){
      pidControl.setReference(angle, ControlType.kPosition);
      climberR.set(0);
      climberL.set(0);
    } else if(!value && !adjust && !limitTop.get() && !limitBottom.get()){
      climberR.set(0);
      climberL.set(0);
    } else if(limitTop.get() || limitBottom.get()){
      climberR.set(0);
      climberL.set(0);
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.getBoolean("Top hit?: ", limitTop.get());
    SmartDashboard.getBoolean("Bottom hit?:", limitBottom.get());
    
  }
}
