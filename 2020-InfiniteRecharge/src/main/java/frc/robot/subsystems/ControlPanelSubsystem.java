package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDMotor;
import frc.robot.Constants;
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

  //  Changed the panelSpinner ID number to a unchangeable variable constant in Constants.java 2/5/20
  private CANSparkMax panelSpinner = new CANSparkMax(Constants.ControlPanelConstants.kpanelSpinnerID, MotorType.kBrushless);
  private PIDMotor panelSpinnerPID = new PIDMotor(panelSpinner, "Panel Spinner");
  private static final double PANEL_SPINNER_SPEED = 0.5; // 0.25 when testing on the bucket
  private CANEncoder panelSpinnerEncoder;
  private static final double SPINNER_RADIUS_INCHES = 2;
  private static final double PANEL_CIRCUMFRENCE_INCHES = 100; // 64 when testing on the bucket
  private static final double GEARBOX_RATIO = 12/1; // motor turns PER axle turn

  private Servo mechanismLifter = new Servo(Constants.ControlPanelConstants.mechanismLifterID);
  private static final double LIFTER_DOWN_ANGLE = 0;
  private static final double LIFTER_UP_ANGLE = 90;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();
  //TUNE THESE COLORS BEFORE MATCHES DURING COMPETITION
  private static final List<Color> CTARGETS_BLUE = new ArrayList<Color>(Arrays.asList(new Color[]{
    ColorMatch.makeColor(0.285, 0.48, 0.22), // Underside, light off, in lab // (0.143, 0.427, 0.429);
    ColorMatch.makeColor(0.17, 0.44, 0.39), // Sideways, light on, thru plexiglass (perpendicular), in lab
    ColorMatch.makeColor(0.12, 0.42, 0.45), // Sideways, light on, thru plexiglass (45* angle), in lab
    ColorMatch.makeColor(0.12, 0.42, 0.46) // Sideways, light on, in lab
  }));
  private static final List<Color> CTARGETS_GREEN = new ArrayList<Color>(Arrays.asList(new Color[]{
    ColorMatch.makeColor(0.325, 0.515, 0.15), // (0.197, 0.561, 0.240);
    ColorMatch.makeColor(0.21, 0.53, 0.26),
    ColorMatch.makeColor(0.16, 0.58, 0.25),
    ColorMatch.makeColor(0.16, 0.59, 0.26)
  }));
  private static final List<Color> CTARGETS_RED = new ArrayList<Color>(Arrays.asList(new Color[]{
    ColorMatch.makeColor(0.585, 0.375, 0.085), // (0.561, 0.232, 0.114);
    ColorMatch.makeColor(0.38, 0.41, 0.21),
    ColorMatch.makeColor(0.50, 0.36, 0.14),
    ColorMatch.makeColor(0.52, 0.34, 0.13)
  }));
  private static final List<Color> CTARGETS_YELLOW = new ArrayList<Color>(Arrays.asList(new Color[]{
    ColorMatch.makeColor(0.43, 0.48, 0.095), // (0.361, 0.524, 0.113);
    ColorMatch.makeColor(0.30, 0.54, 0.16),
    ColorMatch.makeColor(0.32, 0.56, 0.12),
    ColorMatch.makeColor(0.31, 0.56, 0.12)
  }));

  private boolean isMechanismLifted = false;

  public ControlPanelSubsystem() {
    panelSpinnerPID.setOutputRange(-PANEL_SPINNER_SPEED, PANEL_SPINNER_SPEED);

    panelSpinnerEncoder = panelSpinner.getEncoder(); //Sets up the encoder in the motor;
    panelSpinnerPID.dashboardPut();

    // Set up color matcher
    for(Color cTargetBlue : CTARGETS_BLUE)
      colorMatcher.addColorMatch(cTargetBlue);
    for(Color cTargetGreen : CTARGETS_GREEN)
      colorMatcher.addColorMatch(cTargetGreen);
    for(Color cTargetRed : CTARGETS_RED)
      colorMatcher.addColorMatch(cTargetRed);
    for(Color cTargetYellow : CTARGETS_YELLOW)
      colorMatcher.addColorMatch(cTargetYellow);
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
  public void resetPanelSpinnerEncoderPosition()
  {
    panelSpinnerEncoder.setPosition(0);
  }
  public void setPanelSpinnerSpeed(double speed)
  {
    panelSpinner.set(speed * PANEL_SPINNER_SPEED);
  }

  /**
   * @param targetColor The PanelColor to seek toward
   * @return The motor spin direction the subsystem decides on
   */
  public int seekColor(PanelColor targetColor, double speedMulti)
  {
    PanelColor sensorColor = getSensorColor();
    if(sensorColor == null)
      return 0; // No sensor is connected

    //            v   dirToTarget returns the opposite of the direction we want
    int spinDir = - sensorColor.dirToTarget(targetColor);
    panelSpinner.set(PANEL_SPINNER_SPEED * speedMulti * spinDir);
    return spinDir;
  }
  public void overrunInDir(int spinDir, double inches)
  {
    panelSpinnerEncoder.setPosition(0);
    panelSpinnerPID.setPIDPosition(panelSpinnerEncoder.getPosition() + spinDir*inchesToRotations(inches));
  }

  @Override
  public void periodic() {
    getSensorColor(); // Cause color data to be written to dashboard
    panelSpinnerPID.dashboardGet();

    mechanismLifter.set(isMechanismLifted ?LIFTER_UP_ANGLE :LIFTER_DOWN_ANGLE);
  }

  private double inchesToRotations(double inches)
  {
    return (inches / (Math.PI*2*SPINNER_RADIUS_INCHES)) * GEARBOX_RATIO;
  }
  private double rotationsToInches(double rotations)
  {
    return (rotations / GEARBOX_RATIO) * (Math.PI*2*SPINNER_RADIUS_INCHES);
  }

  public PanelColor getSensorColor()
  {
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    PanelColor panelCol = null;
    if(CTARGETS_BLUE.contains(match.color))
        panelCol = PanelColor.BLUE;
    else if(CTARGETS_GREEN.contains(match.color))
      panelCol = PanelColor.GREEN;
    else if(CTARGETS_RED.contains(match.color))
      panelCol = PanelColor.RED;
    else if(CTARGETS_YELLOW.contains(match.color))
      panelCol = PanelColor.YELLOW;
/// SmartDashboard Outputs \\\
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", (panelCol==null) ?"Unknown :(" :panelCol.name);

    return panelCol;
  }

  public boolean isMechanismLifted()
  {
    return isMechanismLifted;
  }
  public void setLiftMechanism(boolean shouldLift)
  {
    isMechanismLifted = shouldLift;
  }
}

