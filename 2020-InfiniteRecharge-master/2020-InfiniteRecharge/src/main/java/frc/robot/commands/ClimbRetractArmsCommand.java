package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbArmsSubsystem;

public class ClimbRetractArmsCommand extends CommandBase {

  private final ClimbArmsSubsystem climber;  

  public ClimbRetractArmsCommand(ClimbArmsSubsystem climber) {
    addRequirements(climber);
    this.climber = climber; // Set variable to the object
  }

  @Override
  public void initialize() {
    climber.retractArms();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    climber.stopArmLeft();
    climber.stopArmRight();
  }

  @Override
  public boolean isFinished() {
    return climber.isClimberArmRetracted();
  }
}
