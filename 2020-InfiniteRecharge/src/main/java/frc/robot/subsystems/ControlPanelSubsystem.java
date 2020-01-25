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

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDMotor;

public class ControlPanelSubsystem extends SubsystemBase {

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
  private static final double PANEL_SPINNER_SPEED = 1;
  private CANEncoder panelSpinnerEncoder;
  private static final double SPINNER_RADIUS_INCHES = 2;
  private static final double PANEL_CIRCUMFRENCE_INCHES = 100;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();
  //TUNE THESE COLORS BEFORE MATCHES DURING COMPETITION
  private static final Color CTARGET_BLUE = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private static final Color CTARGET_GREEN = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private static final Color CTARGET_RED = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private static final Color CTARGET_YELLOW = ColorMatch.makeColor(0.361, 0.524, 0.113);

  private double offsetEncoder = 0;
  private double nowEncoder = 0;

  public ControlPanelSubsystem() {
    panelSpinnerEncoder = panelSpinner.getEncoder(); //Sets up the encoder in the motor;
    panelSpinnerPID.dashboardPut();

    // Set up color matcher
    colorMatcher.addColorMatch(CTARGET_BLUE);
    colorMatcher.addColorMatch(CTARGET_GREEN);
    colorMatcher.addColorMatch(CTARGET_RED);
    colorMatcher.addColorMatch(CTARGET_YELLOW);
  }

  /**
   * Rotate the control panel a certain number of rotations.
   * @param panelRotations The number of times to rotate the control panel.
   */
  public void spinPanel(double panelRotations)
  {
    panelSpinnerEncoder.setPosition(0);
    panelSpinnerPID.setPIDPosition(inchesToRotations(panelRotations * PANEL_CIRCUMFRENCE_INCHES));
  }
  public double getPanelRotations()
  {
    return rotationsToInches(panelSpinnerEncoder.getPosition()) / PANEL_CIRCUMFRENCE_INCHES;
  }

  /**
   * @param targetColor The PanelColor to seek toward
   * @return The motor spin direction the subsystem decides on
   */
  public int seekColor(PanelColor targetColor)
  {
    int spinDir = getSensorColor().dirToTarget(targetColor);
    panelSpinner.set(PANEL_SPINNER_SPEED * spinDir);
    return spinDir;
  }
  public void overrunInDir(int spinDir, double inches)
  {
    panelSpinnerPID.setPIDPosition(panelSpinnerEncoder.getPosition() + spinDir*inchesToRotations(inches));
  }

  @Override
  public void periodic() {
    getSensorColor(); // Cause color data to be written to dashboard
    panelSpinnerPID.dashboardGet();
  }

  private double inchesToRotations(double inches)
  {
    return inches / (Math.PI*2*SPINNER_RADIUS_INCHES);
  }
  private double rotationsToInches(double rotations)
  {
    return rotations * (Math.PI*2*SPINNER_RADIUS_INCHES);
  }

  private PanelColor getSensorColor()
  {
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    PanelColor panelCol = null;
    if(match.color == CTARGET_BLUE)
        panelCol = PanelColor.BLUE;
    else if(match.color == CTARGET_GREEN)
      panelCol = PanelColor.GREEN;
    else if(match.color == CTARGET_RED)
      panelCol = PanelColor.RED;
    else if(match.color == CTARGET_YELLOW)
      panelCol = PanelColor.YELLOW;

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", (panelCol==null) ?"Unknown :(" :panelCol.name);

    return panelCol;
  }
}
