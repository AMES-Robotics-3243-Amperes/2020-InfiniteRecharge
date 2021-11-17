/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.commands.ClimbResetCommand;
//import frc.robot.commands.ClimberJoystickCommand;
import frc.robot.commands.DumperCommand;
import frc.robot.commands.ActuatorCommand;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Auto
  private Command m_autonomousCommand;  // This is a blank command.
  private Command m_shootAutoCommand;
  private Command m_dumpAutoCommand;
  private Command m_doNothingCommand;
  private Command m_lineAutoCommand;
  

  // Tele-op
  private Command m_driveCommand;
  private Command m_dumpCommand;
  private Command m_limelightCommand;
  private Command m_shootCommand;
  //private Command m_climbManualCommand;
  private Command m_ActuatorCommand;
  //private Command m_climbResetCommand;

  private static final String kLineAuto = "Auto Line";
  private static final String kShootAuto = "Auto Shoot";
  private static final String kDumpAuto = "Auto Dump";
  private static final String kNothingAuto = "No Auto";
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    //m_climbResetCommand = new ClimbResetCommand(m_robotContainer.m_climbArmsSubsystem, m_robotContainer.m_climbWinchSubsystem);

    m_shootAutoCommand = m_robotContainer.getShootAutoCommand();  // Shoots and then backs up
    m_dumpAutoCommand = m_robotContainer.getDumpAutoCommand();  // Dumps and then backs up
    m_lineAutoCommand = m_robotContainer.getDriveForwardCommand();  // Drives forward only
    m_doNothingCommand = m_robotContainer.getDoNothingCommand();  // Doesn't do anything for autonomous
    m_chooser.setDefaultOption(kShootAuto, m_shootAutoCommand); // This is the default auto
    m_chooser.addOption(kLineAuto, m_lineAutoCommand);
    m_chooser.addOption(kDumpAuto, m_dumpAutoCommand);
    m_chooser.addOption(kNothingAuto, m_doNothingCommand);
    
    SmartDashboard.putData("Auto Choices", m_chooser);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    
    CommandScheduler.getInstance().run();

    // testMotor.set(0.25 * testJoyst.getRawAxis(1));
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    // Sets the blank command to whatever is chosen in Shuffleboard
    m_autonomousCommand = m_chooser.getSelected();
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule(); // Schedule the autonomous command
    }

    //m_climbResetCommand.schedule(false);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
 /////////////////////////////////////////////////////////////// DECLARING COMMANDS: Declared at the top section //////////////////
    m_driveCommand = m_robotContainer.getDriveCommand(); // Autonomous
    m_limelightCommand = m_robotContainer.getLimelightCommand();
    m_shootCommand = m_robotContainer.getShootCommand();
    //m_climbManualCommand = new ClimberJoystickCommand(m_robotContainer.m_climbArmsSubsystem, m_robotContainer.m_climbWinchSubsystem, m_robotContainer.secondary);
    m_dumpCommand = new DumperCommand();
    m_ActuatorCommand = new ActuatorCommand(m_robotContainer.m_ActuatorSubsystem);
 ////////////////////////////////////////////////////////////////// SCHEDULING COMMANDS ////////////////////////////////
    m_driveCommand.schedule();
    m_shootCommand.schedule();
    //m_climbManualCommand.schedule();
    //m_climbResetCommand.schedule(false);
    m_dumpCommand.schedule();
    m_ActuatorCommand.schedule();

    CommandScheduler.getInstance().run();
////////////////////////////////////////////////////
  }
 /////////////////////////////// vv vv vv vv vv vv |This can be ignored| vv vv vv vv vv vv ///////////////////////////////////////////
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
