package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ClimbWinchSubsystem;
import frc.robot.RobotContainer;

public class ClimbRetractWinchCommand extends CommandBase {

  private final ClimbWinchSubsystem climber;  

  public ClimbRetractWinchCommand(ClimbWinchSubsystem climber) {
    addRequirements(climber);
    this.climber = climber; // Set variable to the object
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if( !climber.isWinchRetracted())
    {
      climber.winchPeriodic(false);
    }

  }

  @Override
  public void end(boolean interrupted) {
    climber.stopWinch();
  }

  @Override
  public boolean isFinished() {
    return climber.isWinchRetracted();
  }
}
