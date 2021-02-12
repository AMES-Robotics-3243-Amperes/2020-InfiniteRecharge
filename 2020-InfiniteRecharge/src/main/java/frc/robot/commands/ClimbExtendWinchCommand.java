package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbWinchSubsystem;

public class ClimbExtendWinchCommand extends CommandBase {

  private final ClimbWinchSubsystem climber;

  public ClimbExtendWinchCommand(ClimbWinchSubsystem climber) {
    addRequirements(climber);
    this.climber = climber; // Set variable to the object
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if( ! climber.isWinchDeployed()){
      climber.winchPeriodic(true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    climber.stopWinch();       
  }

  @Override
  public boolean isFinished() {
    return climber.isWinchDeployed();
  }
}
