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
import frc.robot.util.JoystUtil;
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
  public static Joystick driver = new Joystick(0);
  public static Joystick secondary = new Joystick(1);
  static double steerLeft;
  static double steerRight;

  // This helps the code know if we're using the practice robot or the competition robot
  public static boolean isPractice = Preferences.getInstance().getBoolean("Is Practice", false);
  static boolean toggleActOn = false;
  static boolean toggleActPressed = false;
  
  static boolean toggleSerOn = false;
  static boolean toggleSerPressed = false;

  //-------------------------------------- SUBSYSTEMS --------------------------------------------
  private static IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public static DriveTrainSubSystem m_robotDriveSubsystem = new DriveTrainSubSystem();
  public static ControlPanelSubsystem m_controlPanelSubsystem = new ControlPanelSubsystem();
  public final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  public static DumperSubsystem m_dumperSubsystem = new DumperSubsystem();
  public static ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  //----------------------------------------------------------------------------------------------
  //----------------------------------------------------------- COMMANDS --------------------------------------------------------------
  public final static DriveTrainCommand m_robotDriveCommand = new DriveTrainCommand(m_robotDriveSubsystem);
  private final AutoMoveAndShootCommand m_autoCommand = new AutoMoveAndShootCommand(m_robotDriveSubsystem, m_dumperSubsystem);
  private final LimelightCommand m_limelightCommand = new LimelightCommand(m_robotDriveSubsystem, m_limelightSubsystem);
  protected final ClimbCommand m_climbCommand = new ClimbCommand(m_climbSubsystem,
      new ClimbExtendCommand(m_climbSubsystem), new ClimbRetractCommand(m_climbSubsystem));
  public static IntakeCommand m_IntakeCommand = new IntakeCommand(m_IntakeSubsystem);
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
  public static DriveForward m_driveForward = new DriveForward();
  public static AutoDump m_AutoDump = new AutoDump();
  // -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  // Color Wheel variables. ALSO, YOU CAN PUT THIS INTO CONSTANTS TO MAKE THIS PLACE A LITTLE MORE NEAT?
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

    JoystickButton toggleIntake = new JoystickButton(secondary, 2);
    toggleIntake.toggleWhenPressed(m_IntakeCommand);

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

    steerLeft = JoystUtil.matchZone(steerLeft, sr);
    if (Constants.TEST_VERSION)
      SmartDashboard.putNumber("Unscaled JoyL", steerLeft);
    steerLeft = 0.85 * JoystUtil.scaleZone(steerLeft);

    return steerLeft;
  }

  public static double configureDriveRight() {
    steerRight = getJoystWithDead(false);
    double sl = getJoystWithDead(true);

    steerRight = JoystUtil.matchZone(steerRight, sl);
    if (Constants.TEST_VERSION)
      SmartDashboard.putNumber("Unscaled JoyR", steerRight);
    steerRight = 0.85 * JoystUtil.scaleZone(steerRight);

    return steerRight;
  }

  /** Gets a joystick value, with dead zone applied. */
  private static double getJoystWithDead(boolean isLeft) {
    double steer = isLeft ? driver.getRawAxis(1) : driver.getRawAxis(3);
    steer = JoystUtil.deadZone(steer);
    return steer;
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

  public static boolean configureBallCollectBackwards() {
    return secondary.getRawButton(5);
  }

  public static boolean configureBallShoot(){
    return secondary.getRawButton(8);
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

 
  public Command getIntakeCommand(){

    return m_IntakeCommand;
  }
}
//--------------------------------------------------------------------------------
