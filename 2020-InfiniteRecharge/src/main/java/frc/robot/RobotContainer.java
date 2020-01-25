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
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ControlPanelCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveTrainSubSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveTrainCommand;
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //Defined Suybsystems
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrainSubSystem m_robotDriveSubsystem = new DriveTrainSubSystem();
  public static ControlPanelSubsystem m_controlPanelSubsystem = new ControlPanelSubsystem();
  
  //Defined Commands
  public final DriveTrainCommand m_robotDriveCommand = new DriveTrainCommand(m_robotDriveSubsystem, configureDriveBindings());
  private final ControlPanelCommand m_controlPanelCommand = new ControlPanelCommand(m_controlPanelSubsystem);
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final ControlPanelCommand.TurnToColor turnToColorBlue = new ControlPanelCommand.TurnToColor(m_controlPanelSubsystem, ControlPanelSubsystem.PanelColor.BLUE);
  private final ControlPanelCommand.TurnToColor turnToColorGreen = new ControlPanelCommand.TurnToColor(m_controlPanelSubsystem, ControlPanelSubsystem.PanelColor.GREEN);
  private final ControlPanelCommand.TurnToColor turnToColorRed = new ControlPanelCommand.TurnToColor(m_controlPanelSubsystem, ControlPanelSubsystem.PanelColor.RED);
  private final ControlPanelCommand.TurnToColor turnToColorYellow = new ControlPanelCommand.TurnToColor(m_controlPanelSubsystem, ControlPanelSubsystem.PanelColor.YELLOW);

  //Joysticks
  private final XboxController driver = new XboxController(0);

  //Other variables
  Double[] steering = new Double[2];

  private static final int B_BLUE = 0;
  private static final int B_GREEN = 1;
  private static final int B_RED = 2;
  private static final int B_YELLOW = 3;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

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
    triggerSpinner.toggleWhenPressed(m_controlPanelCommand);  //Whenever you push the button, the referenced command is run
    colorBlue.whenPressed(turnToColorBlue);
    colorGreen.whenPressed(turnToColorGreen);
    colorRed.whenPressed(turnToColorRed);
    colorYellow.whenPressed(turnToColorYellow);
  }
  private Double[] configureDriveBindings(){  //This passes in the axis steering for robot drive

    steering[0] = driver.getRawAxis(1);  //Should be the left axis
    steering[1] = driver.getRawAxis(3);  //Should be the right axis
    
    return steering;
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
}
