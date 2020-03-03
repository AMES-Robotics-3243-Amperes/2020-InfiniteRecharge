package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbWinchSubsystem;
import frc.robot.RobotContainer;

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
    if( ! climber.isWinchDeployed() && ! RobotContainer.getWinchStop())
      climber.winchPeriodic(true);
  }

  @Override
  public void end(boolean interrupted) {
    climber.stopWinch();       
  }

  @Override
  public boolean isFinished() {
    return climber.isWinchDeployed() || RobotContainer.getWinchStop();
  }
}
