package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.RobotContainer;

public class ClimbExtendCommand extends CommandBase {
  
  private final ClimbSubsystem climber;

  public ClimbExtendCommand(ClimbSubsystem climber) {
    addRequirements(climber);
    this.climber = climber;  // Set variable to the object
  }

  @Override
  public void initialize() {
      // TODO
  }

  @Override
  public void execute() {
      // TODO
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ClimbSubsystem.setExtendClimb(0.0, 0.0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
