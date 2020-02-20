/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// Wpilib imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANEncoder;
// SparkMAX imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class AssimilatorSubsystem extends SubsystemBase {
  /**
   * Creates a new AssimilatorSubsystem.
   */
  static CANSparkMax intakeShaft;
  static CANSparkMax intakeActuator;
  static CANSparkMax polyLoop;
  static CANEncoder indexEncoder;
  public static boolean currentExtended = false;
  public static boolean currentRetracted = true;
  static final double CURRENT_CONST = 20.0;

  public AssimilatorSubsystem() {
    intakeShaft = new CANSparkMax(Constants.BallCollectConstants.kSpinID, MotorType.kBrushless);
    intakeActuator = new CANSparkMax(Constants.BallCollectConstants.kActuateID, MotorType.kBrushed);
    intakeShaft.setSmartCurrentLimit(25);
    intakeActuator.setSmartCurrentLimit(28); // Test for limit
    // indexEncoder = intakeActuator.getOutputCurrent();
  }

  /*public static void setIndexCollectSpeed(boolean index) {
    System.out.println(intakeActuator.getOutputCurrent());

    if (index) {
      if (intakeActuator.getOutputCurrent() < CURRENT_CONST && !currentExtended) {
        currentRetracted = false; 
        intakeActuator.set(0.5);
        intakeShaft.set(0.5);
      } else {
        intakeActuator.set(0.0);
        currentExtended = true;
      }
    } else {
      if (intakeActuator.getOutputCurrent() < CURRENT_CONST && !currentRetracted) {
        currentExtended = false;
        intakeActuator.set(-0.5);
        intakeShaft.set(0.0);
      } else{
        currentRetracted = true;
        intakeActuator.stopMotor();
        intakeShaft.stopMotor();

      }
    }
  }*/

  public static void setExtend(){

    System.err.println("It works");
      if (intakeActuator.getOutputCurrent() < CURRENT_CONST && !currentExtended) {
        currentRetracted = false; 
        intakeActuator.set(-0.65);
        System.err.println("E ON");
      } else {
        intakeActuator.set(0.0);
        currentExtended = true;
        System.err.println("E OFF");
      }

  }

  public static void setRetract(){
    System.err.println("## Extend " + Boolean.toString(currentExtended));
    System.err.println("## Retract " + Boolean.toString(currentRetracted));
    System.err.println("#### Current: " + Double.toString(intakeActuator.getOutputCurrent()));
    if (intakeActuator.getOutputCurrent() < CURRENT_CONST && !currentRetracted) {
      currentExtended = false;
      intakeActuator.set(0.65);
      System.err.println("R ON");
    } else{
      currentRetracted = true;
      intakeActuator.stopMotor();
      System.err.println("R OFF");
    }
  }

  @Override
  public void periodic() {

    if(currentExtended){
      intakeShaft.set(0.5);
    } else {
      intakeShaft.stopMotor();
    }
    // This method will be called once per scheduler run
  }
}
