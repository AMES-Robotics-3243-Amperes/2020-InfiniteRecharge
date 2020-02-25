package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.RobotContainer;

public class ClimbExtendCommand extends CommandBase {

  private final ClimbSubsystem climber;
  private boolean hasDeployed; // flag

  public ClimbExtendCommand(ClimbSubsystem climber) {
    addRequirements(climber);
    this.climber = climber; // Set variable to the object
  }

  @Override
  public void initialize() {
    hasDeployed = climber.isWinchDeployed();
  }

  @Override
  public void execute() {

    if (!hasDeployed) {
      climber.extendArmsForClimbing();
      hasDeployed = climber.setExtendWinch();
    }

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return hasDeployed;
  }
}
