/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// Wpilib imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// SparkMAX imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


// ------------------- THIS WAS CHANGED FROM ASSIMILATOR SUBSYSTEM TO INTAKE SUBSYSTEM ---------------------- //

public class IntakeSubsystem extends SubsystemBase {
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

  public IntakeSubsystem() {
    intakeShaft = new CANSparkMax(Constants.BallCollectConstants.kSpinID, MotorType.kBrushless);
    intakeActuator = new CANSparkMax(Constants.BallCollectConstants.kActuateID, MotorType.kBrushed);
    intakeShaft.setSmartCurrentLimit(25);
    intakeActuator.setSmartCurrentLimit(28); // Test for limit
  }

  public static void setExtend(){

      if (intakeActuator.getOutputCurrent() < CURRENT_CONST && !currentExtended) {
        currentRetracted = false; 
        intakeActuator.set(-0.65);
      } else {
        intakeActuator.set(0.0);
        currentExtended = true;
      }

  }

  public static void setRetract(){
    
    if (intakeActuator.getOutputCurrent() < CURRENT_CONST && !currentRetracted) {
      currentExtended = false;
      intakeActuator.set(0.65);
    } else{
      currentRetracted = true;
      intakeActuator.stopMotor();
    }
  }

  @Override
  public void periodic() {

    if(currentExtended){
      intakeShaft.set(-0.5);
    } else {
      intakeShaft.stopMotor();
    }
    // This method will be called once per scheduler run
  }
}
