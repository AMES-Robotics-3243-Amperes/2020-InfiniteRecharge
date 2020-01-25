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
import frc.robot.util.PIDMotor;

public class ControlPanelSubsystem extends SubsystemBase {

  public static enum PanelColor
  {
    RED(0), YELLOW(1), BLUE(2), GREEN(3); // Ordered as they are on the wheel
    public final int value;
    private PanelColor(int value)
    {
      this.value = value;
    }

    public int dirToTarget(PanelColor target)
    {
      int dir = target.value - this.value;

      if(dir < -2)
        dir = 1;
      else if(dir > 2)
        dir = -1;
      else if(dir > 1)
        dir = 1;
      else if (dir < -1)
        dir = -1;

      return dir;
    }
  }

  /**
   * Creates a new ControlPanel.
   */
  private CANSparkMax m_panelSpinner = new CANSparkMax(1, MotorType.kBrushless);
  private PIDMotor panelSpinnerPID = new PIDMotor(m_panelSpinner);
  private static final double PANEL_SPINNER_SPEED = 1;
  private CANEncoder m_trenchEncoder;
  private static final double SPINNER_RADIUS_INCHES = 2;

  private double offsetEncoder = 0;
  private double nowEncoder = 0;

  public ControlPanelSubsystem() {
    m_trenchEncoder = m_panelSpinner.getEncoder(); //Sets up the encoder in the motor;
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
    m_panelSpinner.set(1); //Sets to 100% speed
  }

  public void stop(){
    m_panelSpinner.stopMotor();  //Stops the motor
  }

  /**
   * @param targetColor The PanelColor to seek toward
   * @return The motor spin direction the subsystem decides on
   */
  public int seekColor(PanelColor targetColor)
  {
    int spinDir = PanelColor.RED.dirToTarget(targetColor);
    // TODO: "PanelColor.RED" currently stands in for the PanelColor that should be retrieved from a color sensor subsystem
    m_panelSpinner.set(PANEL_SPINNER_SPEED * spinDir);
    return spinDir;
  }
  public void overrunInDir(int spinDir, double inches)
  {
    panelSpinnerPID.setPIDPosition(m_trenchEncoder.getPosition() + spinDir*inchesToRotations(inches));
  }

  @Override
  public void periodic() {
    
  }

  private double inchesToRotations(double inches)
  {
    return inches / Math.PI*2*SPINNER_RADIUS_INCHES;
  }
}
