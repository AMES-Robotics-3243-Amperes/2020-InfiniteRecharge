/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static Joystick driver = new Joystick(0);
  private static Joystick secondary = new Joystick(1);
  static Double[] steering = new Double[2];
  static double steerLeft;
  static double steerRight;  

  //Defined Suybsystems
  private final AutoSubsystem m_exampleSubsystem = new AutoSubsystem();
  public static DriveTrainSubSystem m_robotDriveSubsystem = new DriveTrainSubSystem();
  public static ControlPanelSubsystem m_controlPanelSubsystem = new ControlPanelSubsystem();
  public final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem(m_robotDriveSubsystem);
  public static BallCollectionSubSystem m_ballCollectionSubsystem = new BallCollectionSubSystem();
  public final DumperSubsystem dumperSubsystem = new DumperSubsystem();
  public static ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  public final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();

  //Defined Commands
  public final DriveTrainCommand m_robotDriveCommand = new DriveTrainCommand(m_robotDriveSubsystem);
  private final AutoCommand m_autoCommand = new AutoCommand(m_exampleSubsystem);
  private final LimelightCommand m_limelightCommand = new LimelightCommand(m_robotDriveSubsystem, m_limelightSubsystem);
  private final BallCollectionCommand m_ballCollectionCommand = new BallCollectionCommand(m_ballCollectionSubsystem, indexerSubsystem);
  public static ClimbCommand m_climbCommand = new ClimbCommand(m_climbSubsystem);

  private final ControlPanelCommand.TurnNumTimes turn4Times = new ControlPanelCommand.TurnNumTimes(m_controlPanelSubsystem, 4);
  private final ControlPanelCommand.TurnToColor turnToColorBlue = new ControlPanelCommand.TurnToColor(m_controlPanelSubsystem, ControlPanelSubsystem.PanelColor.BLUE);
  private final ControlPanelCommand.TurnToColor turnToColorGreen = new ControlPanelCommand.TurnToColor(m_controlPanelSubsystem, ControlPanelSubsystem.PanelColor.GREEN);
  private final ControlPanelCommand.TurnToColor turnToColorRed = new ControlPanelCommand.TurnToColor(m_controlPanelSubsystem, ControlPanelSubsystem.PanelColor.RED);
  private final ControlPanelCommand.TurnToColor turnToColorYellow = new ControlPanelCommand.TurnToColor(m_controlPanelSubsystem, ControlPanelSubsystem.PanelColor.YELLOW);
  private final ControlPanelCommand.LowerMechanism lowerControlPanel = new ControlPanelCommand.LowerMechanism(m_controlPanelSubsystem);

  private final DumperCommand dumperCommand = new DumperCommand(dumperSubsystem, indexerSubsystem, 2);

  //Joysticks

  //Other variables

  private static final int B_BLUE = 1;
  private static final int B_GREEN = 2;
  private static final int B_RED = 3;
  private static final int B_YELLOW = 4;
  private static final int B_TURN_4_TIMES = 10;
  private static final int B_LOWER_CTLPANEL = 9;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Setup Commands
    turnToColorBlue.addRequirements(m_controlPanelSubsystem); // Keep panel rotation cmds from running simultaneously
    turnToColorGreen.addRequirements(m_controlPanelSubsystem);
    turnToColorRed.addRequirements(m_controlPanelSubsystem);
    turnToColorYellow.addRequirements(m_controlPanelSubsystem);
    turn4Times.addRequirements(m_controlPanelSubsystem);

    //Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
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
    // Control panel system is unused
    //JoystickButton bLowerControlPanel = new JoystickButton(driver, B_LOWER_CTLPANEL);
    //bLowerControlPanel.whenPressed(lowerControlPanel);

    JoystickButton dump = new JoystickButton(driver, 6);
    dump.whenPressed(dumperCommand);
  }
  
  //-------------------- LIMELIGHT SECTION OF JOYSTICK ---------------------
  public static boolean driveLime(){
    return driver.getRawButton(8); //I don't know which button to choose yet.
  }

  //-------------------- DRIVING SECTION OF JOYSTICK -----------------------
  public static boolean configurePracBot(){
    return driver.getRawButton(7);  // Don't know which button to choose yet
  }

  public static double configureDriveLeft(){  //This passes in the axis steering for robot drive
    steerLeft = getJoystWithDead(true);  //Should be the left axis
    double sr = getJoystWithDead(false);

    steerLeft = matchZone(steerLeft, sr);
    if(Constants.TEST_VERSION)
      SmartDashboard.putNumber("Unscaled JoyL", steerLeft);
    steerLeft = scaleZone(steerLeft);

    return steerLeft;
  }
  public static double configureDriveRight(){
    steerRight = getJoystWithDead(false);
    double sl = getJoystWithDead(true);
    
    steerRight = matchZone(steerRight, sl);
    if(Constants.TEST_VERSION)
      SmartDashboard.putNumber("Unscaled JoyR", steerRight);
    steerRight = scaleZone(steerRight);

    return steerRight;
  }
  /** Gets a joystick value, with dead zone applied. */
  private static double getJoystWithDead(boolean isLeft)
  {
    double steer = isLeft ?-driver.getRawAxis(1) :-driver.getRawAxis(3);  //Should be the left axis
    steer = deadZone(steer);
    return steer;
  }
  /** Given both left and right steering, returns the average of the two if they're really close together.
   * <p> Useful for trying to drive in a straight line.
   */
  private static double matchZone(double steer1, double steer2)
  {
    double matchZoneRadius = 0.09;

    double avgSteer = (steer1+steer2)/2.0;
    if(Constants.TEST_VERSION)
      SmartDashboard.putNumber("avgSteer", avgSteer);
    double r = matchZoneRadius/2.0;

    // TODO: smoothing
    double lowerBound = (avgSteer-r<=0) ?-1 :0;
    double upperBound = (avgSteer+r>=0) ?0 :1;
    double upperBoundCorrector = clamp(
      (steer1-(avgSteer+r)) * ((r) / (upperBound-(avgSteer+r))),
      0, r
    );
    double lowerBoundCorrector = clamp(
      (steer1-(avgSteer-r)) * ((r) / ((avgSteer-r)-lowerBound)),
      -r, 0
    );
    if(Constants.TEST_VERSION) {
    SmartDashboard.putNumber("upperBoundConnector", upperBoundCorrector);
    SmartDashboard.putNumber("lowerBoundConnector", lowerBoundCorrector); }
    double result = Math.signum(steer1-avgSteer) * Math.max(0, Math.abs(steer1-avgSteer)-r) + avgSteer
      + ((steer1 > avgSteer+r) ?upperBoundCorrector :lowerBoundCorrector);

    return result;
    // return (Math.abs(avgSteer-steer1) <= r) ?avgSteer :steer1; Without smoothing; don't use.
  }
  private static double clamp(double a, double min, double max)
  {
    double realMin = Math.min(min, max);
    double realMax = Math.max(min, max);
    return Math.min(realMax, Math.max(realMin, a));
  }
  private static double lerp(double a, double b, double f)
  {
    return a + f * (b-a);
  }

  public static double deadZone(double dead){      
    double deadZoneRadius = 0.09;
    // This makes input increase smoothly, instead of jumping up slightly as soon as 0.09 is passed.
    return Math.signum(dead) * Math.max(0, Math.abs(dead*(1+deadZoneRadius))-deadZoneRadius);
  }

  public static double scaleZone(double scale){
    return Math.pow(scale, 3);
  }

  //----------------- INDEXER (BALL COLLECTION) SECTION OF JOYSTICK -------------------
  public static boolean configureballbindings(){
    return driver.getRawButton(5);  // Possibly change this to secondary joystick
  }

  //----------------- SHOOTER SECTION OF JOYSTICK --------------------
  public static boolean configureshootbindings(){
    return driver.getRawButton(10); // Possibly change this to secondary joystick
  }

  //--------------- CLIMB GYRO SENSOR SECTION OF JOYSTICK ------------------ 
  public static boolean configureclimbbindings(){
    return driver.getRawButton(9);  //Possibly change this to secondary joystick
  }

  public static boolean configureclimbadjbindings(){
    return driver.getRawButton(8);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public Command getDriveCommand() {

    return m_robotDriveCommand;
  }

  public Command getLimelightCommand(){

    return m_limelightCommand;
  }

  public Command getBallCollectCommand(){

    return m_ballCollectionCommand;
  }

  public Command getClimbCommand(){
    
    return m_climbCommand;
  }
}
