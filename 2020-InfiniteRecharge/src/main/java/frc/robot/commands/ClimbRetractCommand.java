package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.RobotContainer;

public class ClimbRetractCommand extends CommandBase {

  private final ClimbSubsystem climber;  

  public ClimbRetractCommand(ClimbSubsystem climber) {
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
      climber.setWinchIsDeployed(false);
    }

  }

  @Override
  public void end(boolean interrupted) {
    climber.stopAllMotors();
  }

  @Override
  public boolean isFinished() {
    return climber.isWinchRetracted();
  }
}
