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
// SparkMAX imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class AssimilatorSubsystem extends SubsystemBase {
  /**
   * Creates a new AssimilatorSubsystem.
   */
  static CANSparkMax ballIndex;
  static CANSparkMax ballCollect;
  public AssimilatorSubsystem() {
    ballIndex = new CANSparkMax(Constants.BallCollectConstants.kSpinID, MotorType.kBrushless);
    ballCollect = new CANSparkMax(Constants.BallCollectConstants.kActuateID, MotorType.kBrushed);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
