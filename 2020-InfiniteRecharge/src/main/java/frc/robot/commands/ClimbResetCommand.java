package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.RobotContainer;

public class ClimbResetCommand extends CommandBase {
  
  private final ClimbSubsystem climber;
  private boolean isDone = false;
  private boolean  isDoneL = false;
  private boolean isDoneR = false;
  private boolean isDoneW = false; 


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
        isDoneL = climber.resetRetractLeft();
        isDoneR = climber.resetRetractRight();
        isDoneW = climber.resetRetractWinch();

        isDone = isDoneL && isDoneR && isDoneW;
        if(isDone)
        {
            climber.stopAllMotors();
            climber.areEncodersReset = true;
        }
      }
      System.err.println(isDone);
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
