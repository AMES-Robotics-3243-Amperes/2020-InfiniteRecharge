/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
// WPILib Imports ----------------------------------------
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
//--------------------------------------------------------
//Class Imports ------------------------------------------
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.autonomous.*;
//--------------------------------------------------------
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //Joysticks
  private static Joystick driver = new Joystick(0);
  private static Joystick secondary = new Joystick(1);
  static JoystickButton toggleIntake = new JoystickButton(secondary, 2);
  static double steerLeft;
  static double steerRight;

  // This helps the code know if we're using the practice robot or the competition robot
  public static boolean isPractice = Preferences.getInstance().getBoolean("Is Practice", false);
  static boolean toggleActOn = false;
  static boolean toggleActPressed = false;
  
  static boolean toggleSerOn = false;
  static boolean toggleSerPressed = false;

  //-------------------------------------- SUBSYSTEMS --------------------------------------------
  private static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final AutoSubsystem m_exampleSubsystem = new AutoSubsystem();
  public static DriveTrainSubSystem m_robotDriveSubsystem = new DriveTrainSubSystem();
  public static ControlPanelSubsystem m_controlPanelSubsystem = new ControlPanelSubsystem();
  public final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  public static DumperSubsystem m_dumperSubsystem = new DumperSubsystem();
  public static ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  //----------------------------------------------------------------------------------------------
  //----------------------------------------------------------- COMMANDS --------------------------------------------------------------
  public final static DriveTrainCommand m_robotDriveCommand = new DriveTrainCommand(m_robotDriveSubsystem);
  private final AutoCommand m_autoCommand = new AutoCommand(m_exampleSubsystem);
  private final LimelightCommand m_limelightCommand = new LimelightCommand(m_robotDriveSubsystem, m_limelightSubsystem);
  protected final ClimbCommand m_climbCommand = new ClimbCommand(m_climbSubsystem,
      new ClimbExtendCommand(m_climbSubsystem), new ClimbRetractCommand(m_climbSubsystem));
  public static AssimilatorCommand m_AssimilatorCommand = new AssimilatorCommand(m_intakeSubsystem);
  // -------------------------------------------------------------------------------------------------------------------------------------
  // ----------------------------------------------------------- CONTROL PANEL
  // ------------------------------------------------------------------------------------------------------
  private final ControlPanelCommand.TurnNumTimes turn4Times = new ControlPanelCommand.TurnNumTimes(
      m_controlPanelSubsystem, 3.5, 4);
  private final ControlPanelCommand.TurnToColor turnToColorBlue = new ControlPanelCommand.TurnToColor(
      m_controlPanelSubsystem, ControlPanelSubsystem.PanelColor.BLUE);
  private final ControlPanelCommand.TurnToColor turnToColorGreen = new ControlPanelCommand.TurnToColor(
      m_controlPanelSubsystem, ControlPanelSubsystem.PanelColor.GREEN);
  private final ControlPanelCommand.TurnToColor turnToColorRed = new ControlPanelCommand.TurnToColor(
      m_controlPanelSubsystem, ControlPanelSubsystem.PanelColor.RED);
  private final ControlPanelCommand.TurnToColor turnToColorYellow = new ControlPanelCommand.TurnToColor(
      m_controlPanelSubsystem, ControlPanelSubsystem.PanelColor.YELLOW);
  private final ControlPanelCommand.LowerMechanism lowerControlPanel = new ControlPanelCommand.LowerMechanism(
      m_controlPanelSubsystem);
  private final ControlPanelCommand.Manual manualPanelLeft = new ControlPanelCommand.Manual(m_controlPanelSubsystem,
      -0.5);
  private final ControlPanelCommand.Manual manualPanelRight = new ControlPanelCommand.Manual(m_controlPanelSubsystem,
      0.5);

  private final DumperCommand m_dumperCommand = new DumperCommand();
  private static ShootCommand m_shootCommand = new ShootCommand();

  // --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  // --------------------------------------------------------------------
  // AUTONOMOUS
  // ---------------------------------------------------------------------------------------------------------------------------------------------------------------
  public static DriveForward m_driveForward = new DriveForward(/*
                                                                * m_robotDriveSubsystem,
                                                                * DriveTrainSubSystem.getSparkLeft(),
                                                                * DriveTrainSubSystem.getSparkRight(),
                                                                * DriveTrainSubSystem.getVictorLeft(),
                                                                * DriveTrainSubSystem.getVictorRight()
                                                                */ );
  public static AutoDump m_AutoDump = new AutoDump();
  // -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  // Color Wheel variables. ALSO, YOU CAN PUT THIS INTO CONSTANTS TO MAKE THIS
  // PLACE A LITTLE MORE NEAT?
  private static final int B_BLUE = 1;
  private static final int B_GREEN = 2;
  private static final int B_RED = 3;
  private static final int B_YELLOW = 4;
  private static final int B_TURN_4_TIMES = 10;
  private static final int B_LOWER_CTLPANEL = 9;

  // ------------------------------------------------------------------------------------------------------
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // This sets the practice robot you're on to true
    // Preferences.getInstance().putBoolean("Is Practice", true);

    // This says that the robot we're on will pass in true (if it's the prac robot)
    // and falst (if it's the comp robot)

    // Setup Commands
    turnToColorBlue.addRequirements(m_controlPanelSubsystem); // Keep panel rotation cmds from running simultaneously
    turnToColorGreen.addRequirements(m_controlPanelSubsystem);
    turnToColorRed.addRequirements(m_controlPanelSubsystem);
    turnToColorYellow.addRequirements(m_controlPanelSubsystem);
    turn4Times.addRequirements(m_controlPanelSubsystem);

    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // DRIVER
    JoystickButton triggerSpinner = new JoystickButton(driver, 1);

    JoystickButton selectColor = new JoystickButton(driver, 7);
    JoystickButton colorBlue = new JoystickButton(driver, B_BLUE); // TODO: and selectColor
    JoystickButton colorGreen = new JoystickButton(driver, B_GREEN);
    JoystickButton colorRed = new JoystickButton(driver, B_RED);
    JoystickButton colorYellow = new JoystickButton(driver, B_YELLOW);
    JoystickButton turn4TimeButton = new JoystickButton(driver, B_TURN_4_TIMES);

    colorBlue.whenPressed(turnToColorBlue, true); // 'true'=interruptible
    colorGreen.whenPressed(turnToColorGreen, true);
    colorRed.whenPressed(turnToColorRed, true);
    colorYellow.whenPressed(turnToColorYellow, true);
    turn4TimeButton.whenPressed(turn4Times, true);

    JoystickButton blowerControlPanel = new JoystickButton(driver, B_LOWER_CTLPANEL);
    blowerControlPanel.whenPressed(lowerControlPanel);

    POVButton ctlPanelManualLeft = new POVButton(driver, 270);
    POVButton ctlPanelManualRight = new POVButton(driver, 90);
    ctlPanelManualLeft.whenHeld(manualPanelLeft, true);
    ctlPanelManualLeft.whenHeld(manualPanelRight, true);

    JoystickButton dump = new JoystickButton(secondary, 6);
    dump.whenPressed(m_dumperCommand);
    toggleIntake.toggleWhenPressed(m_AssimilatorCommand);

    JoystickButton driveLime = new JoystickButton(driver, 6);
    driveLime.whileActiveOnce(m_limelightCommand);
    driveLime.whenReleased(m_robotDriveCommand);

    // SECONDARY
    JoystickButton climberButton = new JoystickButton(secondary, 4);
    climberButton.whenPressed(m_climbCommand);
  }

  // -------------------- PRIMARY: LIMELIGHT SECTION OF JOYSTICK
  // ---------------------
  public static boolean driveLime() {
    return driver.getRawButton(6);
  }

  // -------------------- PRIMARY: DRIVING SECTION OF JOYSTICK
  // -----------------------
  public static boolean getTurbo() {
    return driver.getRawButton(5);
  }

  public static double configureDriveLeft() { // This passes in the axis steering for robot drive
    steerLeft = getJoystWithDead(true); // Should be the left axis
    double sr = getJoystWithDead(false);

    steerLeft = matchZone(steerLeft, sr);
    if (Constants.TEST_VERSION)
      SmartDashboard.putNumber("Unscaled JoyL", steerLeft);
    steerLeft = scaleZone(steerLeft);

    return steerLeft;
  }

  public static double configureDriveRight() {
    steerRight = getJoystWithDead(false);
    double sl = getJoystWithDead(true);

    steerRight = matchZone(steerRight, sl);
    if (Constants.TEST_VERSION)
      SmartDashboard.putNumber("Unscaled JoyR", steerRight);
    steerRight = scaleZone(steerRight);

    return steerRight;
  }

  /** Gets a joystick value, with dead zone applied. */
  private static double getJoystWithDead(boolean isLeft) {
    double steer = isLeft ? -driver.getRawAxis(1) : -driver.getRawAxis(3); // Should be the left axis
    steer = deadZone(steer);
    return steer;
  }

  /**
   * Given both left and right steering, returns the average of the two if they're
   * really close together.
   * <p>
   * Useful for trying to drive in a straight line.
   */
  private static double matchZone(double steer1, double steer2) {
    double matchZoneRadius = 0.09;

    double avgSteer = (steer1 + steer2) / 2.0;
    if (Constants.TEST_VERSION)
      SmartDashboard.putNumber("avgSteer", avgSteer);
    double r = matchZoneRadius / 2.0;

    // The steer1 values past which steer1's unadjusted value should be returned.
    // If avgSteer is >0, steer1 will be returned at 1 and below 0. If <0, at -1 and
    // above 0.
    double lowerBound = (avgSteer - r <= 0) ? -1 : 0;
    double upperBound = (avgSteer + r >= 0) ? 0 : 1;
    // Very slight bias functions to make sure lowerBound and upperBound are
    // respected.
    double upperBoundCorrector = clamp((steer1 - (avgSteer + r)) * ((r) / (upperBound - (avgSteer + r))), 0, r);
    double lowerBoundCorrector = clamp((steer1 - (avgSteer - r)) * ((r) / ((avgSteer - r) - lowerBound)), -r, 0);

    if (Constants.TEST_VERSION) {
      SmartDashboard.putNumber("upperBoundConnector", upperBoundCorrector);
      SmartDashboard.putNumber("lowerBoundConnector", lowerBoundCorrector);
    }

    // A y=x function with a plateau in the middle. The plateau is at avgSteer and
    // causes values to 'snap' to it.
    // Either lowerBoundCorrector or upperBoundCorrector is added, depending on
    // whether steer1 is between avgSteer
    // and lowerBound, or between avgSteer and upperBound.
    double result = Math.signum(steer1 - avgSteer) * Math.max(0, Math.abs(steer1 - avgSteer) - r) + avgSteer
        + ((steer1 > avgSteer + r) ? upperBoundCorrector : lowerBoundCorrector);

    return result;
    // return (Math.abs(avgSteer-steer1) <= r) ?avgSteer :steer1; Without smoothing;
    // don't use.
  }

  private static double clamp(double a, double min, double max) {
    double realMin = Math.min(min, max);
    double realMax = Math.max(min, max);
    return Math.min(realMax, Math.max(realMin, a));
  }

  private static double lerp(double a, double b, double f) {
    return a + f * (b - a);
  }

  public static double deadZone(double dead) {
    double deadZoneRadius = 0.09;
    // This makes input increase smoothly, instead of jumping up slightly as soon as
    // 0.09 is passed.
    return Math.signum(dead) * Math.max(0, Math.abs(dead * (1 + deadZoneRadius)) - deadZoneRadius);
  }

  public static double scaleZone(double scale) {
    return Math.pow(scale, 3);
  }

  // ----------------- SECONDARY: INTAKE SECTION OF JOYSTICK -------------
  public static boolean configureIndexShaft() {
    return secondary.getRawButton(3);
  }

  // ----------------- SECONDARY: SHOOTER / INDEX SECTION OF JOYSTICK
  // --------------------
  public static boolean configureBallCollect() {
    return secondary.getRawButton(7);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  // ----------------------------- AUTONOMOUS COMMANDS
  // ----------------------------
  public Command getDriveForwardCommand() {
    return m_driveForward;
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public Command getAutoDumpCommand() {

    return m_AutoDump;
  }
  // --------------------------------------------------------------------------------

  public static Command getDriveCommand() { // Drive Train

    return m_robotDriveCommand;
}

public Command getLimelightCommand(){ // Limelight

  return m_limelightCommand;
}

//-------------- BALL COLLECTION / SHOOTER / SECONDARY FUNCTIONS -----------------

  public Command getShootCommand(){
    return m_shootCommand;
  }

  public Command getDumperCommand(){
    return m_dumperCommand;
  }

 
  public Command getAssimilatorCommand(){

    return m_AssimilatorCommand;
  }
}
//--------------------------------------------------------------------------------
