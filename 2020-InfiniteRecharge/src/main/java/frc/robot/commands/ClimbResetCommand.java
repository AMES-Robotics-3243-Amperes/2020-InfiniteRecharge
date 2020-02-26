package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.RobotContainer;

public class ClimbResetCommand extends CommandBase {
  
  private final ClimbSubsystem climber;
  private boolean isDone = false;

  public ClimbResetCommand(ClimbSubsystem climber) {
    addRequirements(climber);
    this.climber = climber;  // Set variable to the object
  }

  @Override
  public void initialize() {
      if( ! isDone)
        climber.areEncodersReset = false;
  }

  @Override
  public void execute() {
      if( ! isDone)
      {
        isDone |= climber.resetRetractLeft() && climber.resetRetractRight() && climber.resetRetractWinch();
        if(isDone)
        {
            climber.stopAllMotors();
            climber.areEncodersReset = true;
        }
      }
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
