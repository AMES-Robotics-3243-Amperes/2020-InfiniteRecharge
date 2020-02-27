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
    hasRetracted = false;
    climber.retractArms();
  }

  @Override
  public void execute() {
    if( ! hasRetracted && climber.isClimberArmRetracted())
    {
      climber.setWinchIsDeployed(false);
      hasRetracted = true;
    }

  }

  @Override
  public void end(boolean interrupted) {
    climber.stopAllMotors();
  }

  @Override
  public boolean isFinished() {
    return climber.isWinchRetracted() && climber.isClimberArmRetracted();
  }
}
