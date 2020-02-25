package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.RobotContainer;

public class ClimbRetractCommand extends CommandBase {

  private final ClimbSubsystem climber;
  private boolean hasRetracted; // flag

  public ClimbRetractCommand(ClimbSubsystem climber) {
    addRequirements(climber);
    this.climber = climber; // Set variable to the object
  }

  @Override
  public void initialize() {
    hasRetracted = climber.isWinchRetracted();
  }

  @Override
  public void execute() {

    if (!hasRetracted) {
      climber.retractArms();
      hasRetracted = climber.setRetractWinch();
    }

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return hasRetracted;
  }
}
