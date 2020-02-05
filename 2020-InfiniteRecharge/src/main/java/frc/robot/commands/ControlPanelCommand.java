/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.ControlPanelSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
/*
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
*/



  public static class TurnNumTimes extends CommandBase
  {
    private final ControlPanelSubsystem controlPanel;
    private final double rotations;

    public TurnNumTimes(ControlPanelSubsystem controlPanel, double rotations)
    {
      this.controlPanel = controlPanel;
      this.rotations = rotations;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      System.out.println("TurnNumTimes initialize");
      controlPanel.spinPanel(rotations);
      SmartDashboard.putString("CtlPanCmd", "TurnNumTimes");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      System.out.println("TurnNumTimes isFinished "+controlPanel.getPanelRotations());
      // Finish when within 1/10th a rotation of the target
      return Math.abs(controlPanel.getPanelRotations() - rotations) <= 0.1;
    } 
  }

  public static class TurnToColor extends CommandBase
  {
    private static final double COLOR_REACHED_STABLE_TIME = 0.5;
    private final ControlPanelSubsystem controlPanel;
    private final ControlPanelSubsystem.PanelColor targetColor;
    private int prevSpinDir = 0;
    private int tryIdx = 0; // How many times have we reached the target color?
    private double lastTimeReachedColor = -10000;
    private boolean isFinished = false;

    public TurnToColor(ControlPanelSubsystem controlPanel, ControlPanelSubsystem.PanelColor targetColor) {
      this.controlPanel = controlPanel;  //Sets the variable to the object
      this.targetColor = targetColor;
      // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      prevSpinDir = 0; // reset
      tryIdx = 0;
      lastTimeReachedColor = Timer.getFPGATimestamp();
      isFinished = false;
      SmartDashboard.putString("CtlPanCmd", "TurnToColor");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      System.out.println("TurnToColor execute");

      if(isFinished)
        return;

      int spinDir = controlPanel.seekColor(targetColor, 1/*Math.pow(0.5, tryIdx)*/);
      SmartDashboard.putNumber("spinDir", spinDir);

      //if(spinDir==0 && prevSpinDir!=spinDir)
      //  tryIdx++;
      
      if(spinDir==0) // If no spin dir, we've reached the target color
      {
        controlPanel.overrunInDir(prevSpinDir, 0);
        //if(Timer.getFPGATimestamp() - lastTimeReachedColor > COLOR_REACHED_STABLE_TIME)
          isFinished = true;
      } else // We're still seeking the color
      {
        // This will start being in the past when spinDir starts being 0
        lastTimeReachedColor = Timer.getFPGATimestamp();
        prevSpinDir = spinDir;
      }

      //prevSpinDir = spinDir;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return isFinished;
    }
  }
}
