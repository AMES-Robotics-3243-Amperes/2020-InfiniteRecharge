/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControlPanelSubsystem extends SubsystemBase {
  /**
   * Creates a new ControlPanel.
   */
  private CANSparkMax m_trenchMotor = new CANSparkMax(1, MotorType.kBrushless);
  private CANEncoder m_trenchEncoder;

  private double offsetEncoder = 0;
  private double nowEncoder = 0;

  public ControlPanelSubsystem() {
    m_trenchEncoder = m_trenchMotor.getEncoder(); //Sets up the encoder in the motor;
  }

  public void getStartPosition(){
    offsetEncoder = m_trenchEncoder.getPosition();  //Gives starting position
  }

  public void getEndPosition(){
    nowEncoder = m_trenchEncoder.getPosition(); //Gets the current position
    nowEncoder = nowEncoder - offsetEncoder;  //Gives the difference (# of revolutions we want to look at)
  }

  public boolean getCheckEnd(){
    return nowEncoder >= 10;  //10 isn't tuned to the right # yet
  }

  public void spin(){
    m_trenchMotor.set(1); //Sets to 100% speed
  }

  public void stop(){
    m_trenchMotor.stopMotor();  //Stops the motor
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
