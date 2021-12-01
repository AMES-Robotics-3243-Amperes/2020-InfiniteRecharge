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
import frc.robot.commands.auto.DoNothingAuto;
import frc.robot.commands.auto.DriveForward;
import frc.robot.commands.auto.DumpCommandGroup;
import frc.robot.commands.auto.ShootCommandGroup;

//--------------------------------------------------------
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {

  // Joysticks
  public static Joystick driver = new Joystick(0);
  public static Joystick secondary = new Joystick(1);
  static double steerLeft;
  static double steerRight;

  // This helps the code know if we're using the practice robot or the competition
  // robot
  // DO NOT CHANGE isPractice TO ANYTHING ELSE OTHER THAN false FOR THE
  // COMPETITION
  public static boolean isPractice = false; // Preferences.getInstance().getBoolean("Is Practice", false);
  static boolean toggleActOn = false;
  static boolean toggleActPressed = false;

  static boolean toggleSerOn = false;
  static boolean toggleSerPressed = false;

  // -------------------------------------- SUBSYSTEMS
  // --------------------------------------------
  private static IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public static DriveTrainSubSystem m_robotDriveSubsystem = new DriveTrainSubSystem();
  public static ControlPanelSubsystem m_controlPanelSubsystem = new ControlPanelSubsystem();
  public final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  public static DumperSubsystem m_dumperSubsystem = new DumperSubsystem();
  // public static ClimbArmsSubsystem m_climbArmsSubsystem = new
  // ClimbArmsSubsystem();
  // public static ClimbWinchSubsystem m_climbWinchSubsystem = new
  // ClimbWinchSubsystem();
  public static ActuatorSubsystem m_ActuatorSubsystem = new ActuatorSubsystem();

  // -------------------------------------- COMMANDS
  // -----------------------------------------------
  public final static DriveTrainCommand m_robotDriveCommand = new DriveTrainCommand(m_robotDriveSubsystem);
  private final LimelightCommand m_limelightCommand = new LimelightCommand(m_robotDriveSubsystem, m_limelightSubsystem);
  // protected final ClimbCommand m_climbCommand = new
  // ClimbCommand(m_climbWinchSubsystem, m_climbArmsSubsystem);
  public static IntakeCommand m_IntakeCommand = new IntakeCommand(m_IntakeSubsystem);
  private static DumperCommand m_dumperCommand = new DumperCommand();
  private ShootCommand m_shootCommand = new ShootCommand(m_dumperSubsystem, m_limelightSubsystem);
  private ActuatorCommand m_ActuatorCommand = new ActuatorCommand(m_ActuatorSubsystem);

  // ------------------------------------ CONTROL PANEL
  // --------------------------------------------
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

  // ------------------------------------- AUTONOMOUS
  // ----------------------------------------------------

  private ShootCommandGroup m_shootAuto = new ShootCommandGroup(m_robotDriveSubsystem, m_limelightSubsystem,
      m_dumperSubsystem);
  private DumpCommandGroup m_dumpAuto = new DumpCommandGroup(m_dumperSubsystem);
  private DoNothingAuto m_doNothing = new DoNothingAuto();
  private DriveForward m_driveForward = new DriveForward(50);

  // -----------------------------------------------------------------------------------------------------

  // Color Wheel variables.
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
    // This says that the robot we're on will pass in true (if it's the prac robot)
    // and false (if it's the comp robot)
    // Preferences.getInstance().putBoolean("Is Practice", true);

    // Setup Commands //////////////// I DON'T KNOW WHAT THIS IS JUNE 2020
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
    JoystickButton colorBlue = new JoystickButton(driver, B_BLUE);
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
    toggleIntake.whenPressed(m_IntakeCommand); // Toggling is done by the command itself

    JoystickButton driveLime = new JoystickButton(driver, 6);
    driveLime.whileActiveOnce(m_limelightCommand);
    driveLime.whenReleased(m_robotDriveCommand);

    // SECONDARY
    JoystickButton climberButton = new JoystickButton(secondary, 4);
    // Starts running ClimbCommand when button 4 is pressed
    // climberButton.whenPressed(m_climbCommand);

    // JoystickButton winchButton = new JoystickButton(secondary, 1);
    // winchButton.cancelWhenPressed(m_climbCommand.extendWinch);
    // winchButton.cancelWhenPressed(m_climbCommand.retractWinch);
    // winchButton.cancelWhenPressed(m_climbCommand);
  }

  public static int cameraPOV() {
    return secondary.getPOV();
  }

  // -------------------- PRIMARY: LIMELIGHT SECTION OF JOYSTICK
  // ---------------------
  public static boolean driveLime() {
    return driver.getRawButton(6);
  }

  // -------------------- PRIMARY: DRIVING SECTION OF JOYSTICK
  // -----------------------
  public static boolean getTurbo() { // GAS GAS GAS : feedback from controller method
    return driver.getRawButton(5);
  }

  private static boolean shouldDriveSlow = false; // initial speed (true from the motor perspective v v v)

  public static boolean getShouldDriveSlow() {
    if (driver.getRawButtonPressed(7))
      shouldDriveSlow = !shouldDriveSlow; // This method is called periodically, so toggling logic will work here.
    return shouldDriveSlow;
  }

  public static double configureDriveLeft() { // This passes in the axis steering for robot drive
                                              // ///////////////////////////////////////////////////////////
    steerLeft = getJoystWithDead(true); // Should be the left axis return driver.getRawButton(5);
    double sr = getJoystWithDead(false);

    steerLeft = JoystUtil.matchZone(steerLeft, sr);
    if (Constants.TEST_VERSION)
      SmartDashboard.putNumber("Unscaled JoyL", steerLeft);
    steerLeft = Constants.DriveConstants.DRIVE_SLOW_SPEED * JoystUtil.scaleZone(steerLeft);

    return steerLeft;
  }

  // REMINDER FOR TOMOROW
  public static double configureDriveRight() {
    steerRight = getJoystWithDead(false);
    double sl = getJoystWithDead(true);

    steerRight = JoystUtil.matchZone(steerRight, sl);
    if (Constants.TEST_VERSION)
      SmartDashboard.putNumber("Unscaled JoyR", steerRight); // Unscaled joystick output
    steerRight = Constants.DriveConstants.DRIVE_SLOW_SPEED * JoystUtil.scaleZone(steerRight); // Scaling done here???

    return steerRight;
  }

  public static boolean configureFastButton() {
    return driver.getRawButton(1); // X-button
  }
  /** ConfigurePov */
  // public static double configurePovHoriz(boolean isHoriPressed){
  // Attempting to create a method to use POV pad like ordinary buttons

  // }

  /** Gets a joystick value, with dead zone applied. */ // DEADZONE JOYSTICK REMINDER//
  private static double getJoystWithDead(boolean isLeft) {
    // Silas: trying using the amount the stick is pressed in any direction times
    // the sign of the vertical axis.
    // Is more intuitive, and may help with perceived stick sensitivity issues.
    double steer = isLeft ? JoystUtil.getAxisMagnitude(driver, 0) * Math.signum(driver.getRawAxis(1)) // Checking if
                                                                                                      // steer signum: 0
                                                                                                      // | 1.0 | -1.0
        : JoystUtil.getAxisMagnitude(driver, 2) * Math.signum(driver.getRawAxis(3));
    steer = JoystUtil.deadZone(steer); // value of double steer is plugged into deadZone, where the final steer output
                                       // is returned
    return steer;
  }

  /** Method below returns signum for POV **/
  // private static double getPovNums(boolean isHoriPressed){ // TO BE DONE. RIGHT
  // NOW JUST GET THE SHOOTER DONE.
  // double Pov = isHoriPressed ? JoystUtil.getPOV(secondary, 5) *
  // Math.signum(secondary.getPOV())
  // }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////
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

  public static boolean configureBallHighShoot() {
    return secondary.getRawButton(6);
  }

  public static boolean configureBallMedShoot() {
    return secondary.getRawButton(1);
  }

  public static boolean configureBallLowShoot() {
    return secondary.getRawButton(8);
  }

  public static boolean actuatorExtend() {
    return secondary.getRawButton(9);
  }

  public static boolean actuatorRetract() {
    return secondary.getRawButton(10);
  }

  public static double actuatorSixty() {
    return secondary.getPOV(0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  // ----------------------------- AUTONOMOUS COMMANDS
  // ----------------------------

  public Command getShootAutoCommand() {
    return m_shootAuto;
  }

  public Command getDumpAutoCommand() {
    return m_dumpAuto;
  }

  public Command getDoNothingCommand() {
    return m_doNothing;
  }

  public Command getDriveForwardCommand() {
    return m_driveForward;
  }

  // ------------------------------------ DRIVING
  // ---------------------------------------

  public static Command getDriveCommand() { // for auto driving

    return m_robotDriveCommand;
  }

  public Command getLimelightCommand() {

    return m_limelightCommand;
  }

  // -------------- BALL COLLECTION / SHOOTER / SECONDARY FUNCTIONS
  // -----------------

  public Command getShootCommand() {
    return m_shootCommand;
  }

  public Command getIntakeCommand() {

    return m_IntakeCommand;
  }
}
