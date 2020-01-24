/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ControlPanelSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControlPanelCommand extends CommandBase {
  /**
   * Creates a new ControlPanelCommand.
   */
  private final ControlPanelSubsystem m_controlPanel;

  public ControlPanelCommand(ControlPanelSubsystem controlPanel) {
    m_controlPanel = controlPanel;  //Sets the variable to the object
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controlPanel.getStartPosition();  //At the beginning of this entire command, we get the encoder's position
    m_controlPanel.spin();  //We start moving the motor.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_controlPanel.getEndPosition();  //Periodically gets the amount of rotations from start to current
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controlPanel.stop();  //After a certain amount of rotations, we stop the motor.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controlPanel.getCheckEnd();  //We see if the motor has rotated 10 revolutions or not.
  }
}
