/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
///       Packages and Imports      \\\
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

//import com.ctre.phoenix.motorcontrol.can.*; For "snow blower" motor that raises the panel spinner

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDMotor;
/*\\\     Packages and Imports      ///*/
public class DumperSubsystem extends SubsystemBase {

  public static enum PanelColor
  {
    RED(0,"Red"), YELLOW(1,"Yellow"), BLUE(2,"Blue"), GREEN(3,"Green"); // Ordered as they are on the wheel
    public final int value;
    public final String name;
    private PanelColor(int value, String name)
    {
      this.value = value;
      this.name = name;
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
  private CANSparkMax panelSpinner = new CANSparkMax(5, MotorType.kBrushless);
  private PIDMotor panelSpinnerPID = new PIDMotor(panelSpinner, "Panel Spinner");
  private static final double PANEL_SPINNER_SPEED = 0.5;
  private CANEncoder panelSpinnerEncoder;
  private static final double SPINNER_RADIUS_INCHES = 2;
  private static final double PANEL_CIRCUMFRENCE_INCHES = 100;
  private static final double GEARBOX_RATIO = 12/1; // motor turns PER axle turn

  //private VictorSPX mechanismLifter

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();
  //TUNE THESE COLORS BEFORE MATCHES DURING COMPETITION
  private static final Color CTARGET_BLUE = ColorMatch.makeColor(0.285, 0.48, 0.22); // (0.143, 0.427, 0.429);
  private static final Color CTARGET_GREEN = ColorMatch.makeColor(0.325, 0.515, 0.15); // (0.197, 0.561, 0.240);
  private static final Color CTARGET_RED = ColorMatch.makeColor(0.585, 0.375, 0.085); // (0.561, 0.232, 0.114);
  private static final Color CTARGET_YELLOW = ColorMatch.makeColor(0.43, 0.48, 0.095); // (0.361, 0.524, 0.113);

  private double offsetEncoder = 0;
  private double nowEncoder = 0;

  public DumperSubsystem() {
    
  }

  @Override
  public void periodic() {
    
  }
}